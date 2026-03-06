package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;
import java.util.function.Consumer;
import frc.robot.constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  // Banderas para no saturar logs cuando driveWithSpeeds falla por NetworkTables o feedforward.
  private boolean m_loggedNetworkTableError = false;
  private boolean m_loggedFFError = false;
  
  /** MapleSim-consistent sim state: wheel positions (m) and heading (rad). Driven by maple-sim physics pose via setSimStateFromMapleSim. */
  private double m_mapleSimLeftPosM = 0.0;
  private double m_mapleSimRightPosM = 0.0;
  private double m_mapleSimHeadingRad = 0.0;
  /** Last physics pose from maple-sim for delta-based wheel position accumulation. Null on first frame or after reset. */
  private Pose2d m_lastMapleSimPose = null;
  /** Called when odometry is reset in sim so maple-sim chassis body can be teleported to the new pose. */
  private Consumer<Pose2d> m_simResetCallback = null;

  // SysId: última tensión aplicada para que el callback de log la registre.
  private volatile double m_sysIdAppliedVoltage = 0.0;
  private final SysIdRoutine m_sysIdRoutine;

  // ===============================
  // Motores (kBrushless para NEO/brushless; kBrushed para brushed — ver DriveConstants)
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

  // No wheel encoders (brushed drivetrain, no budget). In sim, wheel state comes from MapleSim via m_mapleSimLeftPosM/m_mapleSimRightPosM.

  // ===============================
  // Drive (all four motors set explicitly; no leader/follower)
  // ===============================

  private final DifferentialDrive m_drive =
      new DifferentialDrive(this::setLeftOutput, this::setRightOutput);

  // ===============================
  // Odometria
  // ===============================

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

  private final DifferentialDrivePoseEstimator m_poseEstimator;

  public DriveSubsystem() {
    // In sim, encoder/heading state is driven by maple-sim physics via setSimStateFromMapleSim (MapleSimHandler).

    // All four motors configured independently (no leader/follower); DifferentialDrive sets left/right via setLeftOutput/setRightOutput.
    // All drivetrain motors use brake mode (not coast) when idle.
    com.revrobotics.spark.config.SparkMaxConfig driveCfg = new com.revrobotics.spark.config.SparkMaxConfig();
    driveCfg.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    driveCfg.smartCurrentLimit(40);
    driveCfg.openLoopRampRate(0.1);
    driveCfg.voltageCompensation((float) DriveConstants.kNominalVoltage);
    for (SparkMax motor : new SparkMax[] { m_leftFront, m_leftRear, m_rightFront, m_rightRear }) {
      motor.configure(driveCfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
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
  
  // Instanciar feedforward de motor con constantes (valores en DriveConstants).
  // driveWithSpeeds lo usa para calcular tensiones seguras.
  // Mantenemos SimpleMotorFeedforward aquí para que el código use las mismas ganancias que en DriveConstants.
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

  /** Sets both left motors from percent (-1..1) as voltage. Used by DifferentialDrive. */
  private void setLeftOutput(double output) {
    double volts = Math.max(-DriveConstants.kNominalVoltage, Math.min(DriveConstants.kNominalVoltage, output * DriveConstants.kNominalVoltage));
    m_leftFront.setVoltage(volts);
    m_leftRear.setVoltage(volts);
  }

  /** Sets both right motors from percent (-1..1) as voltage. Used by DifferentialDrive. */
  private void setRightOutput(double output) {
    double volts = Math.max(-DriveConstants.kNominalVoltage, Math.min(DriveConstants.kNominalVoltage, output * DriveConstants.kNominalVoltage));
    m_rightFront.setVoltage(volts);
    m_rightRear.setVoltage(volts);
  }

  /** Arcade drive. Fwd and turn are reversed: stick forward → backward, stick right → turn left. */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(
        -fwd * DriveConstants.kDriveSpeedScale,
        -rot * DriveConstants.kTurnSpeedScale);

  // Mantener alimentador del watchdog
  m_drive.feed();
  }

  public void stop() {
    m_drive.stopMotor();
  }

  // ===============================
  // Wheel position/velocity (no encoders: 0 on real; sim uses MapleSim state)
  // ===============================

  public double getLeftAveragePosition() {
    if (RobotBase.isSimulation()) {
      return metersToRotations(m_mapleSimLeftPosM);
    }
    return 0.0;
  }

  public double getRightAveragePosition() {
    if (RobotBase.isSimulation()) {
      return metersToRotations(m_mapleSimRightPosM);
    }
    return 0.0;
  }

  public double getLeftAverageVelocity() {
    return 0.0;
  }

  public double getRightAverageVelocity() {
    return 0.0;
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

    // Actualizar la visualización del campo y registrar la pose para AdvantageKit.
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

  private static double metersToRotations(double meters) {
    double wheelCirc = Math.PI * DriveConstants.kWheelDiameterMeters;
    return meters * DriveConstants.kDriveGearRatio / wheelCirc;
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
    if (RobotBase.isSimulation()) {
      m_mapleSimLeftPosM = 0.0;
      m_mapleSimRightPosM = 0.0;
      m_mapleSimHeadingRad = heading.getRadians();
      m_lastMapleSimPose = pose;
      if (m_simResetCallback != null) {
        m_simResetCallback.accept(pose);
      }
    }

    m_poseEstimator.resetPosition(heading, 0.0, 0.0, pose);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * En simulación, devuelve el rumbo del sim (MapleSim-consistent) para que la odometría sea correcta.
   * En robot real devuelve vacío y se usa el NavX.
   */
  public Optional<Rotation2d> getSimHeading() {
    if (RobotBase.isSimulation()) {
      return Optional.of(Rotation2d.fromRadians(m_mapleSimHeadingRad));
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

  /** On real robot returns (0,0,0) since there are no encoders. In sim returns desired chassis speeds from motor outputs so PathPlanner and projectile sim get a sensible value. */
  public ChassisSpeeds getChassisSpeeds() {
    if (RobotBase.isSimulation()) {
      return getDesiredChassisSpeedsForSim();
    }
    double leftVel = rpmToMetersPerSecond(getLeftAverageVelocity());
    double rightVel = rpmToMetersPerSecond(getRightAverageVelocity());
    double vx = (leftVel + rightVel) / 2.0;
    double omega = (rightVel - leftVel) / DriveConstants.kTrackwidthMeters;
    return new ChassisSpeeds(vx, 0.0, omega);
  }

  /**
   * In simulation, returns desired chassis speeds from current motor outputs (voltage) using the same
   * feedforward model so maple-sim is commanded with what we're trying to do, not encoder-derived speeds.
   */
  public ChassisSpeeds getDesiredChassisSpeedsForSim() {
    if (!RobotBase.isSimulation()) {
      return getChassisSpeeds();
    }
    double leftVolts = m_leftFront.get() * DriveConstants.kNominalVoltage;
    double rightVolts = m_rightFront.get() * DriveConstants.kNominalVoltage;
    double ks = DriveConstants.kDriveKS;
    double kv = DriveConstants.kDriveKV;
    double maxSpeed = DriveConstants.kDriveEstMaxSpeed;
    double leftVel = (Math.abs(leftVolts) > 0.01) ? (leftVolts - Math.signum(leftVolts) * ks) / kv : 0.0;
    double rightVel = (Math.abs(rightVolts) > 0.01) ? (rightVolts - Math.signum(rightVolts) * ks) / kv : 0.0;
    leftVel = Math.max(-maxSpeed, Math.min(maxSpeed, leftVel));
    rightVel = Math.max(-maxSpeed, Math.min(maxSpeed, rightVel));
    double vx = (leftVel + rightVel) / 2.0;
    double omega = (rightVel - leftVel) / DriveConstants.kTrackwidthMeters;
    return new ChassisSpeeds(vx, 0.0, omega);
  }

  /**
   * Updates sim wheel state (heading and left/right distances) from maple-sim physics pose. Call from
   * MapleSimHandler after arena.simulationPeriodic() so drive/odometry reflect the physics body (including collisions).
   * No physical encoders; position is kept in m_mapleSimLeftPosM / m_mapleSimRightPosM for getLeftAveragePosition/getRightAveragePosition in sim.
   */
  public void setSimStateFromMapleSim(Pose2d physicsPose) {
    if (!RobotBase.isSimulation()) return;
    m_mapleSimHeadingRad = physicsPose.getRotation().getRadians();
    if (m_lastMapleSimPose != null) {
      double dx = physicsPose.getX() - m_lastMapleSimPose.getX();
      double dy = physicsPose.getY() - m_lastMapleSimPose.getY();
      double dTheta = physicsPose.getRotation().getRadians() - m_lastMapleSimPose.getRotation().getRadians();
      double arc = Math.hypot(dx, dy);
      double halfTrack = DriveConstants.kTrackwidthMeters / 2.0;
      double leftDelta = arc - dTheta * halfTrack;
      double rightDelta = arc + dTheta * halfTrack;
      m_mapleSimLeftPosM += leftDelta;
      m_mapleSimRightPosM += rightDelta;
    }
    m_lastMapleSimPose = physicsPose;
  }

  /** Registers a callback invoked when odometry is reset in sim so maple-sim chassis can be synced (e.g. teleport body). */
  public void setSimResetCallback(Consumer<Pose2d> callback) {
    m_simResetCallback = callback;
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
    m_leftFront.setVoltage(leftVolts);
    m_leftRear.setVoltage(leftVolts);
    m_rightFront.setVoltage(rightVolts);
    m_rightRear.setVoltage(rightVolts);
    m_drive.feed();
  }
   
  @Override
  public void simulationPeriodic() {
    // Drive sim state is fully driven by maple-sim: MapleSimHandler sets chassis speeds, runs physics,
    // then calls setSimStateFromMapleSim(pose) to sync encoder positions and heading from the physics body.
  }
}