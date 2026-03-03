package frc.robot;

import frc.robot.Camera_Calibration.CameraCalibrationLoader;
import frc.robot.Camera_Calibration.CameraCalibrationLoader.Calibration;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ShooterConstants;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Drv_Commands.DriveCommand;
import frc.robot.commands.Drv_Commands.TurnToAngleCommand;
import frc.robot.commands.Intk_Commands.IntakeCommand;
import frc.robot.commands.Intk_Commands.ToggleIntakeDirectionCommand;
import frc.robot.simulation.SimLaunchNoteCommand;
import frc.robot.simulation.MapleSimHandler;
// IndexerSubsystem eliminado; toda la funcionalidad del indexer fue recortada del proyecto.
import frc.robot.subsystems.TelemetrySubsystem;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;
import edu.wpi.first.math.geometry.Rotation2d;
import java.nio.file.Path;
import java.nio.file.Files;
import java.nio.charset.StandardCharsets;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.UsbAprilTagProcessor;
import frc.robot.commands.Rst_Commands.ResetGyroCommand;
import frc.robot.commands.Rst_Commands.ResetOdometryToVisionCommand;
import frc.robot.commands.Sht_Commands.ShooterCommand;
import frc.robot.commands.Fdr_Commands.FeederCommand;
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


// PathPlanner 2026.1.2
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Wires subsystems, vision, PathPlanner, and all driver/operator bindings.
 * Single place to see which button runs which command. See JUDGES_README.md for a map.
 */
public class RobotContainer {

  // ----- Subsystems -----
  private final NavXSubsystem m_navxSubsystem = new NavXSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final OdometrySubsystem m_odometrySubsystem =
      new OdometrySubsystem(m_driveSubsystem, m_navxSubsystem);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();
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

  // ----- Shuffleboard / PathPlanner -----
  private edu.wpi.first.wpilibj.smartdashboard.SendableChooser<Command> m_ppAutoChooser = null;
  private GenericEntry m_targetXEntry;
  private GenericEntry m_targetYEntry;
  private GenericEntry m_targetHeadingEntry;
  private GenericEntry m_maxSpeedEntry;
  private GenericEntry m_posTolEntry;
  private GenericEntry m_angTolEntry;
  private GenericEntry m_ppltvGainEntry;
  private GenericEntry m_resetOdomEntry;
  private GenericEntry m_shooterRpmEntry;

  public RobotContainer() {
    // Tuning tab and Shooter RPM entry (used by ShooterCommand) must exist before configureBindings().
    var tuningTab = Shuffleboard.getTab("Tuning");
    m_shooterRpmEntry = tuningTab.add("Shooter RPM", ShooterConstants.kShooterMaxRPM * Math.abs(ShooterConstants.kShooterSpeed)).withPosition(8, 0).withSize(2, 1).getEntry();

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
    m_targetXEntry = autoTab.add("Target X (m)", 1.5).getEntry();
    m_targetYEntry = autoTab.add("Target Y (m)", 0.0).getEntry();
    m_targetHeadingEntry = autoTab.add("Target Heading (deg)", 0.0).getEntry();
    m_maxSpeedEntry = autoTab.add("Max Speed (m/s)", 0.6).getEntry();
    m_posTolEntry = autoTab.add("Pos Tol (m)", 0.1).getEntry();
    m_angTolEntry = autoTab.add("Ang Tol (deg)", 5.0).getEntry();
    var resetOdomWidget = autoTab.add("Reiniciar odom al inicio de ruta", true).withPosition(0, 4).withSize(2, 1);
    m_resetOdomEntry = resetOdomWidget.getEntry();

    m_ppltvGainEntry = tuningTab.add("PPLTV dt (s)", 0.02).withPosition(0, 0).withSize(2, 1).getEntry();
    tuningTab.add("Drive KS", DriveConstants.kDriveKS).withPosition(0, 1).withSize(2, 1).getEntry();
    tuningTab.add("Drive KV", DriveConstants.kDriveKV).withPosition(2, 1).withSize(2, 1).getEntry();
    tuningTab.add("Drive KA", DriveConstants.kDriveKA).withPosition(4, 1).withSize(2, 1).getEntry();
    tuningTab.add("Drive Est Max Speed", DriveConstants.kDriveEstMaxSpeed).withPosition(6, 1).withSize(2, 1).getEntry();

    // Mantener estas entradas en la tabla de red; DriveSubsystem las lee directamente.
    tuningTab.add("Use PathPlanner FF", false).withPosition(2, 0).withSize(2, 1).getEntry();
    tuningTab.add("PP FF Scale", 1.0).withPosition(4, 0).withSize(2, 1).getEntry();

    autoTab.addBoolean("Alliance Is Red",
        () -> DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red);

    // ----- PathPlanner: AutoBuilder, logging callbacks, auto chooser on Shuffleboard -----
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      // PPLTVController(dt): dt = paso de discretización en segundos (0.02 = bucle FRC 20 ms). Ver API PathPlanner.
      double ppltvDt = m_ppltvGainEntry.getDouble(0.02);
      if (ppltvDt <= 0 || ppltvDt > 0.1) ppltvDt = 0.02;

      AutoBuilder.configure(
          m_odometrySubsystem::getPose,
          m_odometrySubsystem::resetOdometry,
          m_driveSubsystem::getChassisSpeeds,
          m_driveSubsystem::driveWithSpeeds,
          new PPLTVController(ppltvDt),
          config,
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          m_driveSubsystem);
      Logger.recordOutput("Telemetry/Log", "PathPlanner 2026 fully configured.");

      // Registrar ruta/objetivo en AdvantageKit para que AdvantageScope los muestre en la sección Poses del campo 2D/3D.
      PathPlannerLogging.setLogCurrentPoseCallback(
          pose -> Logger.recordOutput("PathPlanner/CurrentPose", pose));
      PathPlannerLogging.setLogTargetPoseCallback(
          pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));
      PathPlannerLogging.setLogActivePathCallback(
          poses -> Logger.recordOutput("PathPlanner/ActivePath",
              poses.toArray(new edu.wpi.first.math.geometry.Pose2d[0])));

      try {
        m_ppAutoChooser = AutoBuilder.buildAutoChooser();
        autoTab.add("PathPlanner Autos", m_ppAutoChooser).withPosition(4, 3).withSize(3, 2);
        Command exampleCmd = Commands.run(() -> m_driveSubsystem.arcadeDrive(0.4, 0.0), m_driveSubsystem)
            .withTimeout(1.5)
            .finallyDo(() -> m_driveSubsystem.arcadeDrive(0, 0));
        m_ppAutoChooser.setDefaultOption("Ejemplo: recto (programático)", exampleCmd);
      } catch (RuntimeException e) {
        Logger.recordOutput("Telemetry/Errors", "PathPlanner chooser failed: " + e.getMessage());
      }

    } catch (Exception e) {
      Logger.recordOutput("Telemetry/Errors", "PathPlanner config failed: " + e.getMessage());
    }

    // ----- Default command and button bindings -----
    configureBindings();
    m_driveSubsystem.setDefaultCommand(
        new DriveCommand(m_driveSubsystem, m_driverController));
  }

  private void configureBindings() {
    // ----- Driver: SysId (LB + A/B/X/Y), gyro reset (Start), vision reset (Back), turn 90° (X), drive-to-pose (B) -----
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

    // ----- Operator: stop (B), intake (LT), toggle direction (A), unjam (LB), reverse intake (RB), shooter (RT), feeder (Y) -----
    m_operatorController.b()
        .onTrue(new frc.robot.commands.StopMechanismsCommand(
            m_intakeSubsystem, m_shooterSubsystem, m_feederSubsystem));
    m_operatorController.leftTrigger()
        .whileTrue(new IntakeCommand(m_intakeSubsystem));
    m_operatorController.a()
        .onTrue(new ToggleIntakeDirectionCommand(m_intakeSubsystem));
    m_operatorController.leftBumper()
        .onTrue(new frc.robot.commands.Intk_Commands.UnjamCommand(m_intakeSubsystem));
    m_operatorController.rightBumper()
        .whileTrue(new frc.robot.commands.Intk_Commands.ReverseIntakeCommand(m_intakeSubsystem, m_feederSubsystem));
    m_operatorController.rightTrigger()
        .whileTrue(new ShooterCommand(m_shooterSubsystem, m_shooterRpmEntry, m_visionSubsystem));
    m_operatorController.y()
        .whileTrue(new FeederCommand(m_feederSubsystem));

    // Sim only: Driver A (without LB) = launch one FUEL projectile in maple-sim. A+LB = SysId only, no launch.
    m_driverController.a()
        .and(m_driverController.leftBumper().negate())
        .onTrue(new SimLaunchNoteCommand(
            m_driveSubsystem,
            m_odometrySubsystem,
            m_shooterSubsystem));

    // Driver X = girar a 90°. B = conducir hasta pose objetivo (valores en pestaña Autonomous).
    m_driverController.x()
        .onTrue(new TurnToAngleCommand(
            m_driveSubsystem,
            m_navxSubsystem,
            90.0));

    m_driverController.b()
        .onTrue(Commands.runOnce(() -> {
          double x = m_targetXEntry.getDouble(1.5);
          double y = m_targetYEntry.getDouble(0.0);
          double headingDeg = m_targetHeadingEntry.getDouble(0.0);
          double maxSpeed = m_maxSpeedEntry.getDouble(0.6);
          double posTol = m_posTolEntry.getDouble(0.1);
          double angTolDeg = m_angTolEntry.getDouble(5.0);
          edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(
              new frc.robot.commands.Drv_Commands.DriveToPoseCommand(
                  m_driveSubsystem,
                  m_odometrySubsystem,
                  new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)),
                  maxSpeed,
                  posTol,
                  angTolDeg));
        }));
  }

  public Command getAutonomousCommand() {
    if (m_ppAutoChooser == null) {
      return Commands.none();
    }
    Command ppCmd = m_ppAutoChooser.getSelected();
    if (ppCmd == null) {
      return Commands.none();
    }

    m_navxSubsystem.reset();
    try {
      var table = NetworkTableInstance.getDefault()
          .getTable("Shuffleboard").getSubTable("Autonomous").getSubTable("PathPlanner Autos");
      String selected = table.getEntry("selected").getString("");
      if (selected == null || selected.isEmpty()) {
        selected = table.getEntry("default").getString("");
      }
      if (selected != null && !selected.isEmpty()) {
        Path deployAutos = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath()
            .resolve("pathplanner").resolve("autos");
        Path autoFile = null;
        for (String c : new String[] { selected, selected + ".auto", selected + ".auto.json" }) {
          Path p = deployAutos.resolve(c);
          if (Files.exists(p)) {
            autoFile = p;
            break;
          }
        }
        if (autoFile != null && m_resetOdomEntry.getBoolean(true)) {
          String autoJson = Files.readString(autoFile, StandardCharsets.UTF_8);
          if (autoJson.contains("\"resetOdom\": true") || autoJson.contains("\"resetOdom\":true")) {
            Pattern pathNamePattern = Pattern.compile("\"pathName\"\\s*:\\s*\"([^\"]+)\"");
            Matcher pathMatcher = pathNamePattern.matcher(autoJson);
            if (pathMatcher.find()) {
              String pathName = pathMatcher.group(1);
              Path pathFile = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath()
                  .resolve("pathplanner").resolve("paths").resolve(pathName + ".path");
              if (Files.exists(pathFile)) {
                String pathJson = Files.readString(pathFile, StandardCharsets.UTF_8);
                Pattern wpPattern = Pattern.compile("\"waypoints\"\\s*:\\s*\\[([\\s\\S]*?)\\]", Pattern.MULTILINE);
                Matcher wpMatcher = wpPattern.matcher(pathJson);
                if (wpMatcher.find()) {
                  Pattern anchorPattern = Pattern.compile("\"anchor\"\\s*:\\s*\\{[^}]*?\"x\"\\s*:\\s*([0-9.+\\-Eed]+)\\s*,[^}]*?\"y\"\\s*:\\s*([0-9.+\\-Eed]+)");
                  Matcher anchorMatcher = anchorPattern.matcher(wpMatcher.group(1));
                  if (anchorMatcher.find()) {
                    double x = Double.parseDouble(anchorMatcher.group(1));
                    double y = Double.parseDouble(anchorMatcher.group(2));
                    m_odometrySubsystem.resetOdometry(new Pose2d(x, y, new Rotation2d(0.0)));
                    Logger.recordOutput("Telemetry/Log", "[AutoChooser] Reset odometry to path start: " + x + ", " + y);
                  }
                }
              }
            }
          }
        }
      }
    } catch (IOException | RuntimeException e) {
      Logger.recordOutput("Telemetry/Errors", "[AutoChooser] Reset-odom parse error: " + e.getMessage());
      m_odometrySubsystem.resetOdometry(new Pose2d());
    }
    return ppCmd;
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