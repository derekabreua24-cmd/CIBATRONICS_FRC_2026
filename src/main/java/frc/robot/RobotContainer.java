package frc.robot;

import frc.robot.Camera_Calibration.CameraCalibrationLoader;
import frc.robot.Camera_Calibration.CameraCalibrationLoader.Calibration;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ShooterConstants;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Drv_Commands.DriveCommand;
import frc.robot.commands.Drv_Commands.TurnToAngleCommand;
import frc.robot.commands.Intk_Commands.IntakeCommand;
import frc.robot.commands.Intk_Commands.SpitCommand;
import frc.robot.commands.Intk_Commands.ToggleIntakeDirectionCommand;
import frc.robot.commands.Intk_Commands.UnjamCommand;
import frc.robot.simulation.SimLaunchFuelCommand;
import frc.robot.simulation.MapleSimHandler;
// IndexerSubsystem eliminado; toda la funcionalidad del indexer fue recortada del proyecto.
import frc.robot.subsystems.TelemetrySubsystem;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.UsbAprilTagProcessor;
import frc.robot.commands.Rst_Commands.ResetGyroCommand;
import frc.robot.commands.Rst_Commands.ResetOdometryToVisionCommand;
import frc.robot.commands.Sht_Commands.ShooterCommand;
import edu.wpi.first.networktables.GenericEntry;

import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// Se usa referencia completa a SendableChooser más abajo; no importar para evitar avisos de import no usado.
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// DriverStation
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Wires subsystems, vision, and all driver/operator bindings.
 * Single place to see which button runs which command. See JUDGES_README.md for a map.
 */
public class RobotContainer {

  // ----- Subsystems -----
  private final NavXSubsystem m_navxSubsystem = new NavXSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final OdometrySubsystem m_odometrySubsystem =
      new OdometrySubsystem(m_driveSubsystem, m_navxSubsystem);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TelemetrySubsystem m_telemetrySubsystem;

  private VisionSubsystem m_visionSubsystem = null;
  private UsbAprilTagProcessor m_usbProcessor = null;

  private final MapleSimHandler m_mapleSimHandler = new MapleSimHandler();

  // ----- Controllers -----
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // ----- Shuffleboard -----
  private GenericEntry m_resetOdomEntry;
  private GenericEntry m_shooterRpmEntry;

  public RobotContainer() {
    // Tuning tab and Shooter RPM entry (used by ShooterCommand) must exist before configureBindings().
    var tuningTab = Shuffleboard.getTab("Tuning");
    m_shooterRpmEntry = tuningTab.add("Shooter RPM", ShooterConstants.kShooterDefaultRpm).withPosition(8, 0).withSize(2, 1).getEntry();

    // ----- Vision: load AprilTag layout from JSON (no reflection), then start camera processor -----
    try {
      Calibration calib =
          CameraCalibrationLoader.loadFromProperties("camera/camera_calib.properties");

      // Load layout from deploy JSON — prefer 2026 Rebuilt (WPILib standard path first).
      AprilTagFieldLayout fieldLayout = null;
      java.nio.file.Path deploy = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath();
      java.nio.file.Path[] jsonCandidates = new java.nio.file.Path[] {
        deploy.resolve("edu/wpi/first/apriltag/2026-rebuilt-andymark.json"),
        deploy.resolve("apriltagfield_2026.json"),
        deploy.resolve("apriltag_field_2026.json"),
        deploy.resolve("apriltagfield.json")
      };
      for (java.nio.file.Path path : jsonCandidates) {
        if (java.nio.file.Files.exists(path)) {
          try {
            fieldLayout = new AprilTagFieldLayout(path);
            Logger.recordOutput("Telemetry/Log", "AprilTagFieldLayout loaded from JSON: " + path.getFileName());
            break;
          } catch (java.io.IOException e) {
            Logger.recordOutput("Telemetry/Errors", "AprilTagFieldLayout failed to load " + path + ": " + e.getMessage());
          }
        }
      }

      if (fieldLayout == null) {
        Logger.recordOutput("Telemetry/Errors", "Visión deshabilitada: no hay layout de campo AprilTag disponible.");
      } else {
        m_visionSubsystem =
            new VisionSubsystem(fieldLayout, calib.cameraToRobot);

        try {
          m_usbProcessor = new UsbAprilTagProcessor(
              calib.cameraName,
              calib.deviceIndex,
              calib.tagSizeMeters,
              calib.fx,
              calib.fy,
              calib.cx,
              calib.cy,
              m_visionSubsystem,
              calib.resolutionWidth,
              calib.resolutionHeight,
              calib.fps);
          Logger.recordOutput("Telemetry/Log", "Vision successfully initialized (2026 Rebuilt).");
        } catch (UnsatisfiedLinkError | NoClassDefFoundError e) {
          // Cámara/procesador falló (p. ej. en sim sin cámara); mantener VisionSubsystem para inyección sim.
          Logger.recordOutput("Telemetry/Errors", "Cámara/procesador visión no disponible (sim?): " + e.toString());
          m_usbProcessor = null;
        }
      }

      } catch (UnsatisfiedLinkError | NoClassDefFoundError e) {
        Logger.recordOutput("Telemetry/Errors", "Falló la inicialización de visión (nativo/clase faltante): " + e.toString());
        m_visionSubsystem = null;
      }

    m_telemetrySubsystem = new TelemetrySubsystem(
        m_driveSubsystem,
        m_intakeSubsystem,
        m_shooterSubsystem,
        m_driverController,
        m_operatorController,
        m_navxSubsystem,
        m_visionSubsystem);

    // When odometry is reset in sim, sync maple-sim chassis body to the new pose.
    m_driveSubsystem.setSimResetCallback(pose -> m_mapleSimHandler.resetChassisPose(pose));

    var autoTab = Shuffleboard.getTab("Autonomous");
    var resetOdomWidget = autoTab.add("Reiniciar odom al inicio de ruta", true).withPosition(0, 4).withSize(2, 1);
    m_resetOdomEntry = resetOdomWidget.getEntry();

    tuningTab.add("Drive KS", DriveConstants.kDriveKS).withPosition(0, 0).withSize(2, 1).getEntry();
    tuningTab.add("Drive KV", DriveConstants.kDriveKV).withPosition(2, 0).withSize(2, 1).getEntry();
    tuningTab.add("Drive KA", DriveConstants.kDriveKA).withPosition(4, 0).withSize(2, 1).getEntry();
    tuningTab.add("Drive Est Max Speed", DriveConstants.kDriveEstMaxSpeed).withPosition(6, 0).withSize(2, 1).getEntry();

    autoTab.addBoolean("Alliance Is Red",
        () -> DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red);

    // ----- Default command and button bindings -----
    configureBindings();
    m_driveSubsystem.setDefaultCommand(
        new DriveCommand(m_driveSubsystem, m_driverController));
  }

  private void configureBindings() {
    // ----- Driver: SysId (LB + A/B/X/Y), gyro reset (Start), vision reset (Back), turn 90° (X) -----
    // SysId: Left Bumper + A/B/X/Y for drivetrain characterization (see CHARACTERIZATION.md).
    m_driverController.leftBumper().and(m_driverController.a()).whileTrue(
        m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.leftBumper().and(m_driverController.b()).whileTrue(
        m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController.leftBumper().and(m_driverController.x()).whileTrue(
        m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController.leftBumper().and(m_driverController.y()).whileTrue(
        m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_driverController.start()
        .onTrue(new ResetGyroCommand(m_navxSubsystem));

    if (m_visionSubsystem != null) {
      m_driverController.back()
          .onTrue(new ResetOdometryToVisionCommand(
              m_driveSubsystem,
              m_visionSubsystem,
              m_navxSubsystem));
    }

    // ----- Operator: stop (B), intake (LT), toggle direction (A), unjam (LB), shooter (RT) -----
    m_operatorController.b()
        .onTrue(new frc.robot.commands.StopMechanismsCommand(m_intakeSubsystem, m_shooterSubsystem));
    m_operatorController.leftTrigger()
        .whileTrue(new IntakeCommand(m_intakeSubsystem, m_shooterSubsystem));
    m_operatorController.a()
        .onTrue(new ToggleIntakeDirectionCommand(m_intakeSubsystem));
    m_operatorController.leftBumper()
        .onTrue(new frc.robot.commands.Intk_Commands.UnjamCommand(m_intakeSubsystem, m_driveSubsystem));
    m_operatorController.rightTrigger()
        .whileTrue(new ShooterCommand(m_shooterSubsystem, m_shooterRpmEntry, m_intakeSubsystem, m_visionSubsystem));
    m_operatorController.y()
        .whileTrue(new SpitCommand(m_intakeSubsystem, m_shooterSubsystem));

    // Sim only: Driver A (without LB) = launch one FUEL projectile in maple-sim. A+LB = SysId only, no launch.
    m_driverController.a()
        .and(m_driverController.leftBumper().negate())
        .onTrue(new SimLaunchFuelCommand(
            m_driveSubsystem,
            m_odometrySubsystem,
            m_shooterSubsystem));

    // Driver X = turn to 90°.
    m_driverController.x()
        .onTrue(new TurnToAngleCommand(
            m_driveSubsystem,
            m_navxSubsystem,
            90.0));
  }

  public Command getAutonomousCommand() {
    m_navxSubsystem.reset();
    if (m_resetOdomEntry != null && m_resetOdomEntry.getBoolean(true)) {
      m_odometrySubsystem.resetOdometry(new Pose2d());
    }

    // Autonomous: drive back 2 s, unjam (~1.5 s), then shoot for 8 s.
    Command driveBack =
        Commands.run(() -> m_driveSubsystem.arcadeDrive(-0.4, 0.0), m_driveSubsystem)
            .withTimeout(2.0)
            .finallyDo(interrupted -> m_driveSubsystem.stop());
    Command unjam = new UnjamCommand(m_intakeSubsystem, m_driveSubsystem);
    Command shootFor8s =
        Commands.race(
            new ShooterCommand(
                m_shooterSubsystem,
                m_shooterRpmEntry,
                m_intakeSubsystem,
                m_visionSubsystem),
            Commands.waitSeconds(8.0));

    return Commands.sequence(driveBack, unjam, shootFor8s);
  }

  public void shutdownVision() {
    if (m_usbProcessor != null) {
      m_usbProcessor.stop();
    }
  }

  /** Registra un evento en la telemetría (Shuffleboard/AdvantageKit). Usado para logging automático. */
  public void logEvent(String message) {
    m_telemetrySubsystem.logEvent(message);
  }

  /**
   * Resetea la odometría a la pose inicial de simulación definida en {@link frc.robot.constants.MapleSimConstants}.
   * Llamado desde {@link Robot#simulationInit()} para que el robot y maple-sim arranquen en la misma pose.
   */
  public void resetSimulationInitialPose() {
    m_odometrySubsystem.resetOdometry(frc.robot.constants.MapleSimConstants.kSimInitialPose);
  }

  /**
   * Llamado cada ciclo de simulación. Delega en {@link MapleSimHandler} (arena maple-sim,
   * chassis e intake sim, FUEL, inyección de pose en visión).
   */
  public void simulationPeriodic() {
    m_mapleSimHandler.simulationPeriodic(
        m_odometrySubsystem,
        m_visionSubsystem,
        m_driveSubsystem,
        m_intakeSubsystem,
        m_shooterSubsystem);
  }
}