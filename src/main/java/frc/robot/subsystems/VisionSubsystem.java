package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

  /** Una detección de la cámara: ID del tag y transformada tag-a-cámara. */
  public static record VisionDetection(int tagId, Transform3d tagToCamera) {}

  private final AprilTagFieldLayout m_layout;
  private final Transform3d m_cameraToRobot;

  // Almacenamiento seguro para hilos
  private final AtomicReference<Pose2d> m_lastPose = new AtomicReference<>();
  private final AtomicReference<Pose2d[]> m_lastCandidates = new AtomicReference<>();
  private final AtomicReference<Matrix<N3, N1>> m_lastStdDevs = new AtomicReference<>();
  private volatile double m_lastTimestamp = 0.0;
  /** Distancia (m) desde la cámara al tag más cercano en el último frame; &lt; 0 si no hay detección válida. Usado para RPM por distancia del shooter. */
  private volatile double m_lastTargetDistanceMeters = -1.0;

  // ----------------------------------------------------
  // Constructores
  // ----------------------------------------------------
  /** Construye el subsistema con un AprilTagFieldLayout preconstruido y una transformada camara->robot. */
  public VisionSubsystem(AprilTagFieldLayout layout, Transform3d cameraToRobot) {
    m_layout = layout;
    m_cameraToRobot = cameraToRobot;
  }

  /** Constructor con transformada cámara->robot; el llamador debe proporcionar/cargar el layout. */
  public VisionSubsystem(Transform3d cameraToRobot) {
    // Obsoleto: usar el otro constructor con AprilTagFieldLayout.
    throw new IllegalStateException("VisionSubsystem: construir con AprilTagFieldLayout y Transform3d cámara->robot");
  }

  /** Devuelve el AprilTagFieldLayout asociado, si existe. */
  public Optional<AprilTagFieldLayout> getFieldLayout() {
    return Optional.ofNullable(m_layout);
  }

  // ----------------------------------------------------
  // Llamado desde el hilo del procesador UsbAprilTagProcessor
  // ----------------------------------------------------

  /**
   * Procesa varias detecciones de tags de un frame y las fusiona en una sola pose y desviaciones típicas.
   * Usa el timestamp de captura del frame (p. ej. al inicio del bucle de visión) para mejor alineación temporal.
   */
  public Optional<Pose2d> processDetections(List<VisionDetection> detections, double timestampSeconds) {
    if (detections == null || detections.isEmpty()) {
      m_lastTargetDistanceMeters = -1.0;
      return Optional.empty();
    }
    List<Pose2d> validPoses = new ArrayList<>();
    List<Double> distances = new ArrayList<>();
    for (VisionDetection d : detections) {
      Optional<Pose2d> p = computeRobotPose(d.tagId(), d.tagToCamera());
      if (p.isPresent()) {
        validPoses.add(p.get());
        double dist = d.tagToCamera().getTranslation().getNorm();
        distances.add(dist);
      }
    }
    if (validPoses.isEmpty()) {
      return Optional.empty();
    }

    // Fusionar: promedios de x, y y de la rotación (ángulo).
    double sumX = 0, sumY = 0, sumAngle = 0;
    for (Pose2d p : validPoses) {
      sumX += p.getX();
      sumY += p.getY();
      sumAngle += p.getRotation().getRadians();
    }
    int n = validPoses.size();
    double avgX = sumX / n;
    double avgY = sumY / n;
    double avgAngle = sumAngle / n;
    Pose2d fusedPose = new Pose2d(avgX, avgY, new Rotation2d(avgAngle));

    // Desv. tip.: confiar más cuando hay varios tags y están más cerca.
    double avgDist = distances.stream().mapToDouble(Double::doubleValue).average().orElse(0);
    boolean multiTag = n >= 2;
    double stdXY = multiTag ? VisionConstants.kVisionStdDevXYMultiTagClose : VisionConstants.kVisionStdDevXYSingleOrFar;
    if (avgDist > 3.0) {
      stdXY += VisionConstants.kVisionVeryFarStdDevExtra;
    } else if (avgDist > VisionConstants.kVisionFarDistanceMeters) {
      stdXY += VisionConstants.kVisionFarStdDevExtra;
    }
    double stdTheta = multiTag ? VisionConstants.kVisionStdDevThetaMultiTag : VisionConstants.kVisionStdDevThetaSingle;
    Matrix<N3, N1> stdDevs = VecBuilder.fill(stdXY, stdXY, stdTheta);

    m_lastPose.set(fusedPose);
    m_lastTimestamp = timestampSeconds;
    m_lastStdDevs.set(stdDevs);
    double minDist = distances.stream().mapToDouble(Double::doubleValue).min().orElse(-1.0);
    m_lastTargetDistanceMeters = minDist >= 0 ? minDist : -1.0;
    Logger.recordOutput("Vision/RobotPose", fusedPose);
    Logger.recordOutput("Vision/TargetDistanceMeters", m_lastTargetDistanceMeters);

    try {
      Pose2d[] candidates = new Pose2d[] {
        fusedPose,
        fusedPose.transformBy(new Transform2d(new Translation2d(0.1, 0.0), new Rotation2d(0.0))),
        fusedPose.transformBy(new Transform2d(new Translation2d(-0.1, 0.0), new Rotation2d(0.0)))
      };
      m_lastCandidates.set(candidates);
      StringBuilder sb = new StringBuilder();
      sb.append('[');
      for (int i = 0; i < candidates.length; ++i) {
        Pose2d c = candidates[i];
        sb.append('{')
          .append("\"x\":").append(c.getX()).append(',')
          .append("\"y\":").append(c.getY()).append(',')
          .append("\"rotDeg\":").append(c.getRotation().getDegrees())
          .append('}');
        if (i < candidates.length - 1) sb.append(',');
      }
      sb.append(']');
      Logger.recordOutput("Vision/Candidates", sb.toString());
      NetworkTableInstance.getDefault().getTable("Vision").getEntry("Candidates").setString(sb.toString());
    } catch (Throwable t) {
      Logger.recordOutput("Vision/Errors", "VisionSubsystem: publish failed -> " + t.toString());
    }

    return Optional.of(fusedPose);
  }

  /** Ruta de un solo tag; delega en processDetections para consistencia. */
  public Optional<Pose2d> processDetection(
      int tagId,
      Transform3d tagToCamera,
      double timestampSeconds) {
    return processDetections(
        Collections.singletonList(new VisionDetection(tagId, tagToCamera)),
        timestampSeconds);
  }

  // ----------------------------------------------------
  // Calculo principal de pose
  // ----------------------------------------------------
  private Optional<Pose2d> computeRobotPose(
      int tagId,
      Transform3d tagToCamera) {

    Optional<Pose3d> maybeTagPose = m_layout.getTagPose(tagId);
    if (maybeTagPose.isEmpty()) {
      return Optional.empty();
    }

    Pose3d tagPoseField = maybeTagPose.get();

  // pose de la camara en el campo
    Pose3d cameraPoseField =
        tagPoseField.transformBy(tagToCamera);

  // pose del robot en el campo
    Pose3d robotPoseField =
        cameraPoseField.transformBy(m_cameraToRobot.inverse());

    Pose2d robotPose2d = robotPoseField.toPose2d();

    // Filtro basico de sanidad (opcional pero recomendado)
    if (Math.abs(robotPose2d.getX()) > 20 ||
        Math.abs(robotPose2d.getY()) > 20) {
      return Optional.empty();
    }

    return Optional.of(robotPose2d);
  }

  // ----------------------------------------------------
  // Accesores
  // ----------------------------------------------------
  public Optional<Pose2d> getLastPose() {
    return Optional.ofNullable(m_lastPose.get());
  }

  /** Returns the last computed array of Pose2d candidate poses (may be null). */
  public Pose2d[] getLastPoseCandidates() {
    return m_lastCandidates.get();
  }

  public OptionalDouble getLastTimestamp() {
    if (m_lastPose.get() != null) {
      return OptionalDouble.of(m_lastTimestamp);
    }
    return OptionalDouble.empty();
  }

  /** Desv. tip. (x, y, theta) de la última pose de visión fusionada; usar con addVisionMeasurement(pose, tiempo, stdDevs). */
  public Optional<Matrix<N3, N1>> getLastVisionStdDevs() {
    return Optional.ofNullable(m_lastStdDevs.get());
  }

  /** Distancia (m) desde la cámara al tag más cercano en el último frame. Vacío si no hay detección válida. Usar para setVelocitySetpointFromDistanceMeters. */
  public OptionalDouble getLastTargetDistanceMeters() {
    return m_lastTargetDistanceMeters >= 0 ? OptionalDouble.of(m_lastTargetDistanceMeters) : OptionalDouble.empty();
  }

  /**
   * Inyecta pose y distancia sintéticos en simulación (sin cámara) para probar la tubería de visión en AdvantageScope.
   * Solo debe llamarse cuando {@code RobotBase.isSimulation()} es true. Registra Vision/RobotPose y Vision/TargetDistanceMeters.
   */
  public void setSimulationPoseAndDistance(Pose2d pose, double distanceMeters) {
    m_lastPose.set(pose);
    m_lastTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    m_lastTargetDistanceMeters = distanceMeters >= 0 ? distanceMeters : -1.0;
    m_lastStdDevs.set(VecBuilder.fill(
        VisionConstants.kVisionStdDevXYSingleOrFar,
        VisionConstants.kVisionStdDevXYSingleOrFar,
        VisionConstants.kVisionStdDevThetaSingle));
    Logger.recordOutput("Vision/RobotPose", pose);
    Logger.recordOutput("Vision/TargetDistanceMeters", m_lastTargetDistanceMeters);
  }

  @Override
  public void periodic() {
    // Intencionalmente vacío; las actualizaciones de visión se hacen desde un hilo externo.
  }
}