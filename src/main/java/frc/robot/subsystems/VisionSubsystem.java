package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
// AprilTagFields not referenced here; layout is provided by caller

import edu.wpi.first.math.geometry.*;
// Timer not used in this class
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
// DataLogManager usage removed in favor of AdvantageKit Logger

public class VisionSubsystem extends SubsystemBase {

  private final AprilTagFieldLayout m_layout;
  private final Transform3d m_cameraToRobot;

  // Thread-safe storage
  private final AtomicReference<Pose2d> m_lastPose = new AtomicReference<>();
  private final AtomicReference<Pose2d[]> m_lastCandidates = new AtomicReference<>();
  private volatile double m_lastTimestamp = 0.0;

  // ----------------------------------------------------
  // Constructores
  // ----------------------------------------------------
  /** Construye el subsistema con un AprilTagFieldLayout preconstruido y una transformada camara->robot. */
  public VisionSubsystem(AprilTagFieldLayout layout, Transform3d cameraToRobot) {
    m_layout = layout;
    m_cameraToRobot = cameraToRobot;
  }

  /** Construct with a camera->robot transform and let the caller pick/load the layout. */
  public VisionSubsystem(Transform3d cameraToRobot) {
    // Deprecated: caller should provide a layout via the other constructor.
    throw new IllegalStateException("VisionSubsystem: please construct with an AprilTagFieldLayout and a camera->robot Transform3d");
  }

  /** Devuelve el AprilTagFieldLayout asociado, si existe. */
  public Optional<AprilTagFieldLayout> getFieldLayout() {
    return Optional.ofNullable(m_layout);
  }

  // ----------------------------------------------------
  // Llamado desde el hilo del procesador UsbAprilTagProcessor
  // ----------------------------------------------------
  public Optional<Pose2d> processDetection(
      int tagId,
      Transform3d tagToCamera,
      double timestampSeconds) {

    Optional<Pose2d> pose = computeRobotPose(tagId, tagToCamera);

    pose.ifPresent(p -> {
      m_lastPose.set(p);
      m_lastTimestamp = timestampSeconds;
      // Build a small array of candidate poses around the detected pose for debugging
      try {
        Pose2d base = p;
        Pose2d[] candidates = new Pose2d[] {
          base,
          base.transformBy(new Transform2d(new Translation2d(0.1, 0.0), new Rotation2d(0.0))),
          base.transformBy(new Transform2d(new Translation2d(-0.1, 0.0), new Rotation2d(0.0)))
        };
        m_lastCandidates.set(candidates);

        // Publish a compact JSON array to Logger and NetworkTables for AdvantageScope / debugging
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
        try {
          Logger.recordOutput("Vision/Candidates", sb.toString());
        } catch (Throwable t) {
          Logger.recordOutput("Vision/Errors", "VisionSubsystem: Logger publish failed -> " + t.toString());
        }
        try {
          NetworkTableInstance.getDefault().getTable("Vision").getEntry("Candidates").setString(sb.toString());
        } catch (RuntimeException e) {
          Logger.recordOutput("Vision/Errors", "VisionSubsystem: NT publish failed -> " + e.toString());
        }
      } catch (RuntimeException ex) {
        Logger.recordOutput("Vision/Errors", "VisionSubsystem: failed to compute/publish candidates -> " + ex.toString());
      }
    });

    return pose;
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

  @Override
  public void periodic() {
    // Intencionalmente vacio.
    // Las actualizaciones de vision se realizan desde un hilo de procesamiento externo.
  }
}