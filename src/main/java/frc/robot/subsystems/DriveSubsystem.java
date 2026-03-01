package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

// Importaciones para la simulación del tren de rodaje

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.RobotBase;



import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  // Flags to avoid spamming logs from frequently-called driveWithSpeeds when networktables or ff fails
  private boolean m_loggedNetworkTableError = false;
  private boolean m_loggedFFError = false;
  
  // drive sim variables
  
  private DifferentialDrivetrainSim m_driveSim;

  // ===============================
  // Motores
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
  // Encoders
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

  private final NavXSubsystem m_navx;

  public DriveSubsystem(NavXSubsystem navx) {
    m_navx = navx;


  if (RobotBase.isSimulation()) {

    m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getCIM(4),                    
        DriveConstants.kDriveGearRatio,
        2.1,                                  
        7.5,                                 
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
  
  // Instantiate a motor feedforward using constants (placeholder values in Constants)
  // This is primarily used by driveWithSpeeds to compute safe voltages.
  // We keep the SimpleMotorFeedforward here so that runtime code uses the same gains
  // that are documented in Constants.
  // (Note: constants should be replaced with characterization results.)
    
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
  // Per-motor accessors (útiles para telemetría más detallada)
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

    Rotation2d heading;

    if (RobotBase.isSimulation() && m_driveSim != null) {
      // En simulación usamos la orientación proporcionada por el simulador
      heading = m_driveSim.getHeading().times(-1.0); // SimGyro devuelve rotación en sentido antihorario, pero NavX es horario, así que invertimos el signo
    } else {
      // En hardware real usamos la lectura del NavX
      heading = m_navx.getRotation2d();
    }

    // Actualizar el estimador de pose con la orientación y las distancias de rueda

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

  public void updateOdometry(Rotation2d heading) {
    double leftMeters = rotationsToMeters(getLeftAveragePosition());
    double rightMeters = rotationsToMeters(getRightAveragePosition());
    m_poseEstimator.update(heading, leftMeters, rightMeters);
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

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
  }

  public ChassisSpeeds getChassisSpeeds() {

  // Convert SparkMax RPM to meters per second
  double leftVel = rpmToMetersPerSecond(getLeftAverageVelocity());
  double rightVel = rpmToMetersPerSecond(getRightAverageVelocity());

  double vx = (leftVel + rightVel) / 2.0;
  double omega = (rightVel - leftVel) / DriveConstants.kTrackwidthMeters;

  return new ChassisSpeeds(vx, 0.0, omega);
}

  /**
   * BiConsumer target for PathPlanner's AutoBuilder: accept desired chassis speeds
   * and optional DriveFeedforwards (ignored here) and command voltages to the motors.
   */
  public void driveWithSpeeds(ChassisSpeeds speeds, DriveFeedforwards ff) {
    // Read live tuning values from NetworkTables if present (Shuffleboard Tuning tab).
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

    // Compute base voltages using our feedforward model from DrivePhysics.
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

    // Optionally incorporate PathPlanner-provided DriveFeedforwards if present and enabled
    try {
      boolean usePPFF = NetworkTableInstance.getDefault()
          .getEntry("/Shuffleboard/Tuning/Use PathPlanner FF").getBoolean(false);
      double ppScale = NetworkTableInstance.getDefault()
          .getEntry("/Shuffleboard/Tuning/PP FF Scale").getDouble(1.0);

      if (usePPFF && ff != null) {
        // PathPlanner provides arrays of accelerations (m/s^2). We'll take the first
        // available acceleration value as a conservative estimate of current accel.
        double acc = 0.0;
        double[] accs = ff.accelerationsMPSSq();
        if (accs != null && accs.length > 0) {
          acc = accs[0] * ppScale;
        }

        // Add the acceleration feedforward component (ka * acc) to both sides.
        leftVolts += DriveConstants.kDriveKA * acc;
        rightVolts += DriveConstants.kDriveKA * acc;
      }
    } catch (RuntimeException e) {
      if (!m_loggedFFError) {
  Logger.recordOutput("Telemetry/Errors", "DriveSubsystem: failed to apply PathPlanner feedforward -> " + e.toString());
        m_loggedFFError = true;
      }
      // Non-fatal otherwise
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

    // Feed motor outputs into sim (convert percent to volts)
    m_driveSim.setInputs(
        m_leftGroup.get() * 12.0,
        m_rightGroup.get() * 12.0
    );

    m_driveSim.update(0.02);

    // Get simulated positions in meters
    double leftMeters = m_driveSim.getLeftPositionMeters();
    double rightMeters = m_driveSim.getRightPositionMeters();

    // Convert meters to motor rotations
    double wheelCirc = Math.PI * DriveConstants.kWheelDiameterMeters;

    double leftRotations =
        (leftMeters / wheelCirc) * DriveConstants.kDriveGearRatio;

    double rightRotations =
        (rightMeters / wheelCirc) * DriveConstants.kDriveGearRatio;

    // Set SparkMax encoder positions manually
    m_leftFrontEncoder.setPosition(leftRotations);
    m_leftRearEncoder.setPosition(leftRotations);
    m_rightFrontEncoder.setPosition(rightRotations);
    m_rightRearEncoder.setPosition(rightRotations);
}
}