package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.DrivePhysics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

// Importaciones para la simulación del tren de rodaje

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  // Banderas para no saturar logs cuando driveWithSpeeds falla por NetworkTables o feedforward.
  private boolean m_loggedNetworkTableError = false;
  private boolean m_loggedFFError = false;
  
  // Variables de simulación del tren de rodaje.
  private DifferentialDrivetrainSim m_driveSim;

  // SysId: última tensión aplicada para que el callback de log la registre.
  private volatile double m_sysIdAppliedVoltage = 0.0;
  private final SysIdRoutine m_sysIdRoutine;

  // ===============================
  // Motores (kBrushless para NEO/brushless; kBrushed para brushed — ver Constants)
  // ===============================

  private final SparkMax m_leftFront =
    new SparkMax(DriveConstants.kLeftFrontMotorPort, MotorType.kBrushed);

  private final SparkMax m_leftRear =
    new SparkMax(DriveConstants.kLeftRearMotorPort, MotorType.kBrushed);

  private final SparkMax m_rightFront =
    new SparkMax(DriveConstants.kRightFrontMotorPort, MotorType.kBrushed);

  private final SparkMax m_rightRear =
    new SparkMax(DriveConstants.kRightRearMotorPort, MotorType.kBrushed);

  private final Field2d m_field = new Field2d();

  // ===============================
  // Codificadores
  // ===============================

  private final RelativeEncoder m_leftFrontEncoder = m_leftFront.getEncoder();
  private final RelativeEncoder m_leftRearEncoder = m_leftRear.getEncoder();
  private final RelativeEncoder m_rightFrontEncoder = m_rightFront.getEncoder();
  private final RelativeEncoder m_rightRearEncoder = m_rightRear.getEncoder();

  // ===============================
  // Grupos de motores
  // ===============================

  @SuppressWarnings("removal")
  private final MotorControllerGroup m_leftGroup =
      new MotorControllerGroup(m_leftFront, m_leftRear);

  @SuppressWarnings("removal")
  private final MotorControllerGroup m_rightGroup =
      new MotorControllerGroup(m_rightFront, m_rightRear);

  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_leftGroup, m_rightGroup);

  // ===============================
  // Odometria
  // ===============================

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

  private final DifferentialDrivePoseEstimator m_poseEstimator;

  public DriveSubsystem() {
    if (RobotBase.isSimulation()) {

    m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getCIM(4),
        DriveConstants.kDriveGearRatio,
        DriveConstants.kSimMomentOfInertiaKgM2,
        DriveConstants.kSimMassKg,
        DriveConstants.kWheelDiameterMeters / 2.0,
        DriveConstants.kTrackwidthMeters,
        null
    );
}


  SmartDashboard.putData("Field", m_field);

  // Seguridad de motores (motor safety)
  m_drive.setSafetyEnabled(true);
  m_drive.setExpiration(0.1);
  
  // Inicializar estimador de pose
    m_poseEstimator =
        new DifferentialDrivePoseEstimator(
            m_kinematics,
            Rotation2d.fromDegrees(0.0),
            rotationsToMeters(getLeftAveragePosition()),
            rotationsToMeters(getRightAveragePosition()),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, 0.1),
            VecBuilder.fill(0.5, 0.5, 0.5));
  
  // Instanciar feedforward de motor con constantes (valores provisionales en Constants).
  // driveWithSpeeds lo usa para calcular tensiones seguras.
  // Mantenemos SimpleMotorFeedforward aquí para que el código use las mismas ganancias que en Constants.
  // Sustituir constantes por resultados de caracterización.

  // Rutina SysId para caracterización del tren de rodaje: misma tensión a ambos lados, registrar posición/velocidad media en m, m/s.
  m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> {
                double v = volts.baseUnitMagnitude();
                m_sysIdAppliedVoltage = v;
                tankDriveVolts(v, v);
              },
              log -> {
                double leftPosM = rotationsToMeters(getLeftAveragePosition());
                double rightPosM = rotationsToMeters(getRightAveragePosition());
                double leftVelMps = rpmToMetersPerSecond(getLeftAverageVelocity());
                double rightVelMps = rpmToMetersPerSecond(getRightAverageVelocity());
                log.motor("Drive")
                    .voltage(Units.Volts.of(m_sysIdAppliedVoltage))
                    .linearPosition(Units.Meters.of((leftPosM + rightPosM) / 2.0))
                    .linearVelocity(
                        Units.MetersPerSecond.of((leftVelMps + rightVelMps) / 2.0));
              },
              this,
              "Drive"));
  }

  // ===============================
  // Métodos de conducción (teleop / básicos)
  // ===============================

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(
        fwd * DriveConstants.kDriveSpeedScale,
        rot * DriveConstants.kTurnSpeedScale);

  // Mantener alimentador del watchdog
  m_drive.feed();
  }

  public void stop() {
    m_drive.stopMotor();
  }

  // ===============================
  // Lectores de encoder
  // ===============================

  public double getLeftAveragePosition() {
    return (m_leftFrontEncoder.getPosition()
        + m_leftRearEncoder.getPosition()) / 2.0;
  }

  public double getRightAveragePosition() {
    return (m_rightFrontEncoder.getPosition()
        + m_rightRearEncoder.getPosition()) / 2.0;
  }

  public double getLeftAverageVelocity() {
    return (m_leftFrontEncoder.getVelocity()
        + m_leftRearEncoder.getVelocity()) / 2.0;
  }

  public double getRightAverageVelocity() {
    return (m_rightFrontEncoder.getVelocity()
        + m_rightRearEncoder.getVelocity()) / 2.0;
  }

  public double getLeftTotalCurrent() {
    return m_leftFront.getOutputCurrent()
        + m_leftRear.getOutputCurrent();
  }

  public double getRightTotalCurrent() {
    return m_rightFront.getOutputCurrent()
        + m_rightRear.getOutputCurrent();
  }

  public double getLeftAverage() {
    return (m_leftFront.get() + m_leftRear.get()) / 2.0;
  }

  public double getRightAverage() {
    return (m_rightFront.get() + m_rightRear.get()) / 2.0;
  }

  // ===============================
  // Accesores por motor (útiles para telemetría más detallada)
  // ===============================

  public double getLeftFrontCurrent() {
    return m_leftFront.getOutputCurrent();
  }

  public double getLeftRearCurrent() {
    return m_leftRear.getOutputCurrent();
  }

  public double getRightFrontCurrent() {
    return m_rightFront.getOutputCurrent();
  }

  public double getRightRearCurrent() {
    return m_rightRear.getOutputCurrent();
  }

  public double getLeftFrontOutput() {
    return m_leftFront.get();
  }

  public double getLeftRearOutput() {
    return m_leftRear.get();
  }

  public double getRightFrontOutput() {
    return m_rightFront.get();
  }

  public double getRightRearOutput() {
    return m_rightRear.get();
  }

  // Voltaje y temperatura por motor
  public double getLeftFrontVoltage() {
    return m_leftFront.getBusVoltage();
  }

  public double getLeftRearVoltage() {
    return m_leftRear.getBusVoltage();
  }

  public double getRightFrontVoltage() {
    return m_rightFront.getBusVoltage();
  }

  public double getRightRearVoltage() {
    return m_rightRear.getBusVoltage();
  }

  public double getLeftFrontTemperature() {
    return m_leftFront.getMotorTemperature();
  }

  public double getLeftRearTemperature() {
    return m_leftRear.getMotorTemperature();
  }

  public double getRightFrontTemperature() {
    return m_rightFront.getMotorTemperature();
  }

  public double getRightRearTemperature() {
    return m_rightRear.getMotorTemperature();
  }

  // ===============================
  // Periodic
  // Se ejecuta cada ciclo del robot; actualiza la odometría y publica telemetría
  // ===============================

  @Override
  public void periodic() {
    // La pose se actualiza en OdometrySubsystem.periodic() con updateOdometryWithTime(); aquí solo se muestra.
    Pose2d pose = m_poseEstimator.getEstimatedPosition();

    // Actualizar la visualización del campo y registrar la pose para AdvantageKit
    m_field.setRobotPose(pose);
    Logger.recordOutput("Odometry/Robot", pose);

    // Publicar valores principales en SmartDashboard/Shuffleboard
    SmartDashboard.putNumber("Robot X", pose.getX());
    SmartDashboard.putNumber("Robot Y", pose.getY());
    SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
  }


  // ===============================
  // Odometría
  // ===============================

  private static double rotationsToMeters(double rotations) {
    double wheelCirc = Math.PI * DriveConstants.kWheelDiameterMeters;
    return rotations * wheelCirc / DriveConstants.kDriveGearRatio;
  }

  private static double rpmToMetersPerSecond(double rpm) {
  double wheelCirc = Math.PI * DriveConstants.kWheelDiameterMeters;
  double rps = rpm / 60.0;
  return (rps * wheelCirc) / DriveConstants.kDriveGearRatio;
}

  /** Actualiza la pose con encoders y giro. Llamar en cada ciclo del robot (p. ej. desde OdometrySubsystem). */
  public void updateOdometry(Rotation2d heading) {
    double leftMeters = rotationsToMeters(getLeftAveragePosition());
    double rightMeters = rotationsToMeters(getRightAveragePosition());
    m_poseEstimator.update(heading, leftMeters, rightMeters);
  }

  /**
   * Igual que updateOdometry pero con timestamp explícito para compensar latencia de medidas de visión.
   * Usar al fusionar visión para que los timestamps de addVisionMeasurement usen la misma fuente de tiempo.
   */
  public void updateOdometryWithTime(double currentTimeSeconds, Rotation2d heading) {
    double leftMeters = rotationsToMeters(getLeftAveragePosition());
    double rightMeters = rotationsToMeters(getRightAveragePosition());
    m_poseEstimator.updateWithTime(currentTimeSeconds, heading, leftMeters, rightMeters);
  }

  public void resetOdometry(Pose2d pose, Rotation2d heading) {

  // Reiniciar posiciones de encoder a cero antes de resetear odometría
  m_leftFrontEncoder.setPosition(0);
    m_leftRearEncoder.setPosition(0);
    m_rightFrontEncoder.setPosition(0);
    m_rightRearEncoder.setPosition(0);

    m_poseEstimator.resetPosition(heading, 0.0, 0.0, pose);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * En simulación, devuelve el rumbo del sim del tren de rodaje para que la odometría sea correcta.
   * Si no hay sim o es null, devuelve vacío.
   */
  public Optional<Rotation2d> getSimHeading() {
    if (m_driveSim != null) {
      return Optional.of(m_driveSim.getHeading());
    }
    return Optional.empty();
  }

  /** Prueba SysId cuasiestática; asignar a un botón y mantener. Ver CHARACTERIZATION.md. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /** Prueba SysId dinámica; asignar a un botón y mantener. Ver CHARACTERIZATION.md. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
  }

  /** Añade una medida de visión con desviaciones típicas por medida (m, m, rad). */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, visionMeasurementStdDevs);
  }

  public ChassisSpeeds getChassisSpeeds() {

  // Convertir RPM de SparkMax a metros por segundo.
  double leftVel = rpmToMetersPerSecond(getLeftAverageVelocity());
  double rightVel = rpmToMetersPerSecond(getRightAverageVelocity());

  double vx = (leftVel + rightVel) / 2.0;
  double omega = (rightVel - leftVel) / DriveConstants.kTrackwidthMeters;

  return new ChassisSpeeds(vx, 0.0, omega);
}

  /**
   * Destino BiConsumer del AutoBuilder de PathPlanner: acepta velocidades deseadas del chasis
   * y opcionalmente DriveFeedforwards (ignorado aquí) y aplica tensiones a los motores.
   */
  public void driveWithSpeeds(ChassisSpeeds speeds, DriveFeedforwards ff) {
    // Leer valores de afinado en vivo desde NetworkTables si existen (pestaña Tuning de Shuffleboard).
    double ks = DriveConstants.kDriveKS;
    double kv = DriveConstants.kDriveKV;
    double ka = DriveConstants.kDriveKA;
    double estMax = DriveConstants.kDriveEstMaxSpeed;
    try {
      ks = NetworkTableInstance.getDefault().getEntry("/Shuffleboard/Tuning/Drive KS").getDouble(ks);
      kv = NetworkTableInstance.getDefault().getEntry("/Shuffleboard/Tuning/Drive KV").getDouble(kv);
      ka = NetworkTableInstance.getDefault().getEntry("/Shuffleboard/Tuning/Drive KA").getDouble(ka);
      estMax = NetworkTableInstance.getDefault().getEntry("/Shuffleboard/Tuning/Drive Est Max Speed").getDouble(estMax);
    } catch (RuntimeException e) {
      if (!m_loggedNetworkTableError) {
  Logger.recordOutput("Telemetry/Errors", "DriveSubsystem: failed to read NetworkTables tuning entries -> " + e.toString());
        m_loggedNetworkTableError = true;
      }
    }

    // Calcular tensiones base con el modelo de feedforward de DrivePhysics.
    double[] volts = DrivePhysics.computeTankVoltages(
        speeds.vxMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        ks,
        kv,
        ka,
        DriveConstants.kTrackwidthMeters,
        estMax);

    double leftVolts = volts[0];
    double rightVolts = volts[1];

    // Opcionalmente incorporar DriveFeedforwards de PathPlanner si está presente y habilitado.
    try {
      boolean usePPFF = NetworkTableInstance.getDefault()
          .getEntry("/Shuffleboard/Tuning/Use PathPlanner FF").getBoolean(false);
      double ppScale = NetworkTableInstance.getDefault()
          .getEntry("/Shuffleboard/Tuning/PP FF Scale").getDouble(1.0);

      if (usePPFF && ff != null) {
        // PathPlanner proporciona arrays de aceleraciones (m/s²); usamos la primera como estimación conservadora.
        double acc = 0.0;
        double[] accs = ff.accelerationsMPSSq();
        if (accs != null && accs.length > 0) {
          acc = accs[0] * ppScale;
        }

        // Añadir el componente de feedforward por aceleración (ka * acc) a ambos lados.
        leftVolts += DriveConstants.kDriveKA * acc;
        rightVolts += DriveConstants.kDriveKA * acc;
      }
    } catch (RuntimeException e) {
      if (!m_loggedFFError) {
  Logger.recordOutput("Telemetry/Errors", "DriveSubsystem: failed to apply PathPlanner feedforward -> " + e.toString());
        m_loggedFFError = true;
      }
      // No es fatal en otro caso.
    }

    tankDriveVolts(leftVolts, rightVolts);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
  m_leftGroup.setVoltage(leftVolts);
  m_rightGroup.setVoltage(rightVolts);
  m_drive.feed();
  }
   
  @Override
  public void simulationPeriodic() {
    if (m_driveSim == null) return;

    // setInputs espera voltios; get() es salida en porcentaje [-1,1], multiplicar por 12 (WPILib 2026).
    m_driveSim.setInputs(
        m_leftGroup.get() * 12.0,
        m_rightGroup.get() * 12.0
    );
    m_driveSim.update(0.02);

    // El sim devuelve distancias de rueda en metros; convertir a rotaciones de motor para encoders SparkMax.
    double leftMeters = m_driveSim.getLeftPositionMeters();
    double rightMeters = m_driveSim.getRightPositionMeters();
    double wheelCirc = Math.PI * DriveConstants.kWheelDiameterMeters;
    double leftRotations = (leftMeters / wheelCirc) * DriveConstants.kDriveGearRatio;
    double rightRotations = (rightMeters / wheelCirc) * DriveConstants.kDriveGearRatio;

    m_leftFrontEncoder.setPosition(leftRotations);
    m_leftRearEncoder.setPosition(leftRotations);
    m_rightFrontEncoder.setPosition(rightRotations);
    m_rightRearEncoder.setPosition(rightRotations);

    // Giro del sim: OdometrySubsystem usa getSimHeading() en simulación para que la pose sea correcta.
  }
}