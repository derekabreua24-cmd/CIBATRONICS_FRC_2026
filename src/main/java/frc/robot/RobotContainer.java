package frc.robot;

import frc.robot.Camera_Calibration.CameraCalibrationLoader;
import frc.robot.Camera_Calibration.CameraCalibrationLoader.Calibration;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Drv_Commands.DriveCommand;
import frc.robot.commands.Drv_Commands.TurnToAngleCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.commands.IndexerCommand;
import frc.robot.subsystems.TelemetrySubsystem;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.subsystems.UsbAprilTagProcessor;
import frc.robot.commands.Rst_Commands.ResetGyroCommand;
import frc.robot.commands.Rst_Commands.ResetOdometryToVisionCommand;
import frc.robot.commands.LogEventCommand;

import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// using fully-qualified SendableChooser reference below; keep import removed to avoid unused-import warnings
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// DriverStation
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// PathPlanner 2026.1.2
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TelemetrySubsystem m_telemetrySubsystem;
  private final NavXSubsystem m_navxSubsystem = new NavXSubsystem();
  private final OdometrySubsystem m_odometrySubsystem =
      new OdometrySubsystem(m_driveSubsystem, m_navxSubsystem);

  private final frc.robot.subsystems.OperatorSubsystem m_operatorSubsystem =
      new frc.robot.subsystems.OperatorSubsystem();

  private VisionSubsystem m_visionSubsystem = null;
  private UsbAprilTagProcessor m_usbProcessor = null;

    private edu.wpi.first.wpilibj.smartdashboard.SendableChooser<Command> m_ppAutoChooser = null;

  @SuppressWarnings("unused")
private GenericEntry m_targetXEntry;
  @SuppressWarnings("unused")
private GenericEntry m_targetYEntry;
  @SuppressWarnings("unused")
private GenericEntry m_targetHeadingEntry;
  @SuppressWarnings("unused")
private GenericEntry m_maxSpeedEntry;
  @SuppressWarnings("unused")
private GenericEntry m_posTolEntry;
  @SuppressWarnings("unused")
private GenericEntry m_angTolEntry;
    private GenericEntry m_ppltvGainEntry;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {

    configureBindings();

    m_driveSubsystem.setDefaultCommand(
        new DriveCommand(m_driveSubsystem, m_driverController));

    try {

      Calibration calib =
          CameraCalibrationLoader.loadFromProperties(
              "camera/camera_calib.properties");

      edu.wpi.first.apriltag.AprilTagFieldLayout fieldLayout =
          edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
              edu.wpi.first.apriltag.AprilTagFields.kDefaultField);

      m_visionSubsystem =
          new VisionSubsystem(fieldLayout, calib.cameraToRobot);

      m_usbProcessor = new UsbAprilTagProcessor(
          calib.cameraName,
          calib.deviceIndex,
          calib.tagSizeMeters,
          calib.fx,
          calib.fy,
          calib.cx,
          calib.cy,
          m_visionSubsystem);

  edu.wpi.first.wpilibj.Filesystem.getDeployDirectory();
  edu.wpi.first.wpilibj.DataLogManager.log("Vision successfully initialized (2026 Rebuilt).");

      } catch (UnsatisfiedLinkError | NoClassDefFoundError e) {
        // Native library load failures (UnsatisfiedLinkError) or missing classes should
        // be handled gracefully in simulation; log and continue with vision disabled.
        edu.wpi.first.wpilibj.DataLogManager.log("Vision failed to initialize (native/missing class): " + e.toString());
        m_visionSubsystem = null;
      }

    m_telemetrySubsystem = new TelemetrySubsystem(
        m_driveSubsystem,
        m_intakeSubsystem,
        m_indexerSubsystem,
        m_shooterSubsystem,
        m_driverController,
        m_operatorController,
        m_navxSubsystem,
        m_visionSubsystem);

    var autoTab = Shuffleboard.getTab("Autonomous");
    var tuningTab = Shuffleboard.getTab("Tuning");

    m_targetXEntry = autoTab.add("Target X (m)", 1.5).getEntry();
    m_targetYEntry = autoTab.add("Target Y (m)", 0.0).getEntry();
    m_targetHeadingEntry = autoTab.add("Target Heading (deg)", 0.0).getEntry();
    m_maxSpeedEntry = autoTab.add("Max Speed (m/s)", 0.6).getEntry();
    m_posTolEntry = autoTab.add("Pos Tol (m)", 0.1).getEntry();
    m_angTolEntry = autoTab.add("Ang Tol (deg)", 5.0).getEntry();

    // Tuning entries (live-adjustable gains)
    m_ppltvGainEntry = tuningTab.add("PPLTV Gain", 1.0).withPosition(0, 0).withSize(2, 1).getEntry();
    // Drive feedforward live-tuning entries (used by DriveSubsystem if present)
    tuningTab.add("Drive KS", DriveConstants.kDriveKS).withPosition(0, 1).withSize(2, 1).getEntry();
    tuningTab.add("Drive KV", DriveConstants.kDriveKV).withPosition(2, 1).withSize(2, 1).getEntry();
    tuningTab.add("Drive KA", DriveConstants.kDriveKA).withPosition(4, 1).withSize(2, 1).getEntry();
    tuningTab.add("Drive Est Max Speed", DriveConstants.kDriveEstMaxSpeed).withPosition(6, 1).withSize(2, 1).getEntry();

    // Keep these entries created on the network table; DriveSubsystem will read them directly.
    tuningTab.add("Use PathPlanner FF", false).withPosition(2, 0).withSize(2, 1).getEntry();
    tuningTab.add("PP FF Scale", 1.0).withPosition(4, 0).withSize(2, 1).getEntry();

    // Legacy string-based chooser removed; PathPlanner chooser is primary.

    autoTab.addBoolean("Alliance Is Red",
        () -> DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red);

    m_driverController.start()
        .onTrue(new ResetGyroCommand(m_navxSubsystem));

    if (m_visionSubsystem != null) {
      m_driverController.back()
          .onTrue(new ResetOdometryToVisionCommand(
              m_driveSubsystem,
              m_visionSubsystem,
              m_navxSubsystem));
    }

    m_driverController.y()
        .onTrue(new LogEventCommand(
            m_telemetrySubsystem,
            "Manual event: Driver Y pressed"));

    m_operatorController.b()
        .onTrue(new frc.robot.commands.ToggleOperatorModeCommand(m_operatorSubsystem));

    m_operatorController.y()
        .onTrue(new LogEventCommand(
            m_telemetrySubsystem,
            "Manual event: Operator Y pressed"));

    // ================= PATHPLANNER 2026.1.2 CONFIG =================
    try {

      RobotConfig config = RobotConfig.fromGUISettings();

      double ppltvGain = m_ppltvGainEntry.getDouble(1.0);

      AutoBuilder.configure(
    m_odometrySubsystem::getPose,              // ✅ Pose supplier
    m_odometrySubsystem::resetOdometry,        // ✅ Reset pose
    m_driveSubsystem::getChassisSpeeds,
    m_driveSubsystem::driveWithSpeeds,
    new PPLTVController(ppltvGain),
    config,
    () -> DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red,
    m_driveSubsystem
);
        edu.wpi.first.wpilibj.DataLogManager.log("PathPlanner 2026 fully configured.");

            // Build the PathPlanner auto chooser and show it on Shuffleboard
            try {
                m_ppAutoChooser = AutoBuilder.buildAutoChooser();
                        autoTab.add("PathPlanner Autos", m_ppAutoChooser).withPosition(4, 3).withSize(3, 2);

                        // Add a simple programmatic example to the PathPlanner chooser so there's
                        // always at least one selectable option for smoke testing. Make it the
                        // default so the simulator/DS will pick it automatically if nothing else
                        // is selected from the GUI.
                        try {
                            Command exampleCmd = Commands.run(() -> m_driveSubsystem.arcadeDrive(0.4, 0.0), m_driveSubsystem)
                                    .withTimeout(1.5)
                                    .finallyDo(() -> m_driveSubsystem.arcadeDrive(0, 0));

                            // Make this the default option so it is selected by default in the chooser
                            m_ppAutoChooser.setDefaultOption("Example: Straight (programmatic)", exampleCmd);
            } catch (RuntimeException e) {
              edu.wpi.first.wpilibj.DataLogManager.log("PathPlanner chooser: failed to add example option -> " + e.toString());
            }
      } catch (RuntimeException e) {
        edu.wpi.first.wpilibj.DataLogManager.log("Failed to build PathPlanner auto chooser: " + e.getMessage());
      }

    } catch (Exception e) {

  edu.wpi.first.wpilibj.DataLogManager.log("PathPlanner configuration failed: " + e.getMessage());

    }
    // ================================================================

  }

  private void configureBindings() {

    m_operatorController.rightBumper()
        .whileTrue(new IntakeCommand(m_intakeSubsystem));

    m_operatorController.leftBumper()
        .whileTrue(new IndexerCommand(m_indexerSubsystem));

    m_operatorController.a()
        .whileTrue(new frc.robot.commands.ShootSequenceCommand(
            m_shooterSubsystem,
            m_indexerSubsystem,
            m_intakeSubsystem));

    m_driverController.x()
        .onTrue(new TurnToAngleCommand(
            m_driveSubsystem,
            m_navxSubsystem,
            90.0));
  }

  public Command getAutonomousCommand() {

    
        // If PathPlanner chooser is available and has a selected command, use it.
        if (m_ppAutoChooser != null) {
            Command ppCmd = m_ppAutoChooser.getSelected();
            if (ppCmd != null) {
                // reset sensors/odometry before running the selected auto
                m_navxSubsystem.reset();
                m_odometrySubsystem.resetOdometry(new Pose2d());
                return ppCmd;
            }
        }

        // No PathPlanner auto selected; return no-op
        return Commands.none();
  }

  public void shutdownVision() {
    if (m_usbProcessor != null) {
      m_usbProcessor.stop();
    }
  }
}