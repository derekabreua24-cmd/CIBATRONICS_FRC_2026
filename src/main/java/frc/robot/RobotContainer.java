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
// IndexerSubsystem removed — all indexer functionality trimmed from project
import frc.robot.subsystems.TelemetrySubsystem;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
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
import frc.robot.commands.LogEventCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ToggleIntakeDirectionCommand;
import edu.wpi.first.networktables.GenericEntry;

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

  private final NavXSubsystem m_navxSubsystem = new NavXSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_navxSubsystem);
  private final OdometrySubsystem m_odometrySubsystem =
      new OdometrySubsystem(m_driveSubsystem, m_navxSubsystem);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TelemetrySubsystem m_telemetrySubsystem;
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
    private GenericEntry m_resetOdomEntry;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  // Shooter RPM tuning entry (adjustable at runtime)
  private GenericEntry m_shooterRpmEntry;

  public RobotContainer() {

    configureBindings();

    m_driveSubsystem.setDefaultCommand(
        new DriveCommand(m_driveSubsystem, m_driverController));

    try {

      Calibration calib =
          CameraCalibrationLoader.loadFromProperties(
              "camera/camera_calib.properties");

      // Prefer a deploy-time APRILTAG JSON if present (e.g. apriltagfield_2026.json)
      edu.wpi.first.apriltag.AprilTagFieldLayout fieldLayout = null;
      try {
        java.nio.file.Path deploy = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath();
        java.nio.file.Path[] candidates = new java.nio.file.Path[] {
          deploy.resolve("apriltagfield_2026.json"),
          deploy.resolve("apriltagfield.json"),
          deploy.resolve("apriltag_field_2026.json")
        };

        java.nio.file.Path found = null;
        for (java.nio.file.Path p : candidates) {
          if (java.nio.file.Files.exists(p)) {
            found = p;
            break;
          }
        }

        if (found != null) {
          // Use reflection to call AprilTagFieldLayout.loadFromFile(File) if available.
          try {
            Class<?> cls = Class.forName("edu.wpi.first.apriltag.AprilTagFieldLayout");
            java.lang.reflect.Method m = cls.getMethod("loadFromFile", java.io.File.class);
            Object obj = m.invoke(null, found.toFile());
            if (obj instanceof edu.wpi.first.apriltag.AprilTagFieldLayout) {
              fieldLayout = (edu.wpi.first.apriltag.AprilTagFieldLayout) obj;
              Logger.recordOutput("Telemetry/Log", "Loaded AprilTag field layout from deploy: " + found.toString());
            }
          } catch (Throwable t) {
            // reflection failed; fall back below
            Logger.recordOutput("Telemetry/Errors", "AprilTagFieldLayout.loadFromFile reflection failed: " + t.toString());
          }
        }

        if (fieldLayout == null) {
          // Fallback to built-in resource (may be older). Keep existing behavior.
          fieldLayout = edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
              edu.wpi.first.apriltag.AprilTagFields.kDefaultField);
          Logger.recordOutput("Telemetry/Log", "Using built-in AprilTagFieldLayout (default)");
        }
        else {
          // If we successfully loaded a built-in layout (e.g., 2026), but no deploy
          // JSON was present, serialize the layout to deploy so users can inspect or
          // ship it to robot. This is non-fatal and best-effort.
          try {
            java.nio.file.Path exportPath = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath().resolve("apriltagfield_2026.json");
            if (!java.nio.file.Files.exists(exportPath)) {
              fieldLayout.serialize(exportPath);
              Logger.recordOutput("Telemetry/Log", "Exported AprilTagFieldLayout to deploy: " + exportPath.toString());
            }
          } catch (Throwable t) {
            Logger.recordOutput("Telemetry/Errors", "Failed to export AprilTagFieldLayout to deploy: " + t.toString());
          }
        }
      } catch (Throwable t) {
        // Defensive fallback: if anything goes wrong, try to load the default field layout
        try {
          fieldLayout = edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
              edu.wpi.first.apriltag.AprilTagFields.kDefaultField);
        } catch (Throwable t2) {
          Logger.recordOutput("Telemetry/Errors", "Failed to initialize AprilTagFieldLayout: " + t2.toString());
          fieldLayout = null;
        }
      }

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
  Logger.recordOutput("Telemetry/Log", "Vision successfully initialized (2026 Rebuilt).");

      } catch (UnsatisfiedLinkError | NoClassDefFoundError e) {
        // Native library load failures (UnsatisfiedLinkError) or missing classes should
        // be handled gracefully in simulation; log and continue with vision disabled.
  Logger.recordOutput("Telemetry/Errors", "Vision failed to initialize (native/missing class): " + e.toString());
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

  // (m_shooterRpmEntry is created below once tuningTab is available)

    var autoTab = Shuffleboard.getTab("Autonomous");
    var tuningTab = Shuffleboard.getTab("Tuning");

  // Add a tuning entry for shooter RPM on the Tuning tab (runtime-adjustable)
  m_shooterRpmEntry = tuningTab.add("Shooter RPM", DriveConstants.kShooterMaxRPM * Math.abs(DriveConstants.kShooterSpeed)).withPosition(8, 0).withSize(2, 1).getEntry();

  m_targetXEntry = autoTab.add("Target X (m)", 1.5).getEntry();
    m_targetYEntry = autoTab.add("Target Y (m)", 0.0).getEntry();
    m_targetHeadingEntry = autoTab.add("Target Heading (deg)", 0.0).getEntry();
    m_maxSpeedEntry = autoTab.add("Max Speed (m/s)", 0.6).getEntry();
    m_posTolEntry = autoTab.add("Pos Tol (m)", 0.1).getEntry();
    m_angTolEntry = autoTab.add("Ang Tol (deg)", 5.0).getEntry();

  // Opción para controlar si queremos que el robot reinicie la odometría al inicio
  // del auto seleccionado (cuando el archivo .auto tiene resetOdom=true).
  var resetOdomWidget = autoTab.add("Reset odom to path start", true).withPosition(0, 4).withSize(2, 1);
  m_resetOdomEntry = resetOdomWidget.getEntry();

  // Entradas de afinación (ganancias ajustables en tiempo de ejecución)
    m_ppltvGainEntry = tuningTab.add("PPLTV Gain", 1.0).withPosition(0, 0).withSize(2, 1).getEntry();
  // Entradas de feedforward de la unidad de tracción (usadas por DriveSubsystem si están habilitadas)
    tuningTab.add("Drive KS", DriveConstants.kDriveKS).withPosition(0, 1).withSize(2, 1).getEntry();
    tuningTab.add("Drive KV", DriveConstants.kDriveKV).withPosition(2, 1).withSize(2, 1).getEntry();
    tuningTab.add("Drive KA", DriveConstants.kDriveKA).withPosition(4, 1).withSize(2, 1).getEntry();
    tuningTab.add("Drive Est Max Speed", DriveConstants.kDriveEstMaxSpeed).withPosition(6, 1).withSize(2, 1).getEntry();

    // Keep these entries created on the network table; DriveSubsystem will read them directly.
    tuningTab.add("Use PathPlanner FF", false).withPosition(2, 0).withSize(2, 1).getEntry();
    tuningTab.add("PP FF Scale", 1.0).withPosition(4, 0).withSize(2, 1).getEntry();

  // El selector legacy basado en cadenas fue removido; el selector de PathPlanner es el principal.

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
  Logger.recordOutput("Telemetry/Log", "PathPlanner 2026 fully configured.");

            // Construir el selector de autos de PathPlanner y mostrarlo en Shuffleboard
            try {
                m_ppAutoChooser = AutoBuilder.buildAutoChooser();
                        autoTab.add("PathPlanner Autos", m_ppAutoChooser).withPosition(4, 3).withSize(3, 2);

                        // Añadir un ejemplo programático simple al selector de PathPlanner para
                        // asegurar que siempre haya al menos una opción seleccionable para pruebas
                        // rápidas (smoke tests). Lo establecemos como predeterminado para que el
                        // simulador/Driver Station lo elija automáticamente si no se selecciona
                        // otra opción en la GUI.
                        try {
                            Command exampleCmd = Commands.run(() -> m_driveSubsystem.arcadeDrive(0.4, 0.0), m_driveSubsystem)
                                    .withTimeout(1.5)
                                    .finallyDo(() -> m_driveSubsystem.arcadeDrive(0, 0));

                            // Make this the default option so it is selected by default in the chooser
                            m_ppAutoChooser.setDefaultOption("Example: Straight (programmatic)", exampleCmd);
            } catch (RuntimeException e) {
              Logger.recordOutput("Telemetry/Errors", "PathPlanner chooser: failed to add example option -> " + e.toString());
            }
      } catch (RuntimeException e) {
  Logger.recordOutput("Telemetry/Errors", "Failed to build PathPlanner auto chooser: " + e.getMessage());
      }

    } catch (Exception e) {
      // RobotConfig.fromGUISettings may throw checked parsing exceptions from
      // the PathPlanner config loader; keep a broad catch here to avoid
      // crashing robot initialization while logging the root cause.
  Logger.recordOutput("Telemetry/Errors", "PathPlanner configuration failed: " + e.getMessage());
    }
    // ================================================================

  }

  private void configureBindings() {

  // Intake on left trigger (runs in configured direction; toggle direction below)
  m_operatorController.leftTrigger()
    .whileTrue(new IntakeCommand(m_intakeSubsystem));

  // Toggle intake direction on 'A' button (press to flip forward/back)
  m_operatorController.a()
    .onTrue(new ToggleIntakeDirectionCommand(m_intakeSubsystem));

  // Shooter spin while operator holds right trigger (RPM adjustable via Tuning tab)
  m_operatorController.rightTrigger()
    .whileTrue(new ShooterCommand(m_shooterSubsystem, m_shooterRpmEntry));

  // ShootSequenceCommand removed (indexer removed). If you want a simple shooter+intake
  // hold command, we can add it here later.

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
                // reiniciar sensores/odometría antes de ejecutar el auto seleccionado
                m_navxSubsystem.reset();
        // Attempt to read the chooser's selected name from Shuffleboard's NetworkTables
                try {
          var table = NetworkTableInstance.getDefault()
            .getTable("Shuffleboard")
            .getSubTable("Autonomous")
            .getSubTable("PathPlanner Autos");

          String selected = table.getEntry("selected").getString("");
          if (selected == null || selected.isEmpty()) {
            // Some SendableChooser variants prefix with the title; try a fallback
            selected = table.getEntry("default").getString("");
          }

                    if (selected != null && !selected.isEmpty()) {
            // Try common filename patterns for autos in deploy/PathPlanner/autos
            Path deployAutos = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath().resolve("PathPlanner").resolve("autos");
            Path autoFile = null;
            String[] candidates = new String[] { selected, selected + ".auto", selected + ".auto.json" };
            for (String c : candidates) {
              Path p = deployAutos.resolve(c);
              if (Files.exists(p)) {
                autoFile = p;
                break;
              }
            }

            if (autoFile != null) {
              String autoJson = Files.readString(autoFile, StandardCharsets.UTF_8);
              boolean resetOdom = autoJson.contains("\"resetOdom\": true") || autoJson.contains("\"resetOdom\":true");
              // Only perform the automatic odometry reset if the user enabled
              // the Shuffleboard toggle for it.
              boolean userRequested = m_resetOdomEntry.getBoolean(true);
              if (resetOdom && userRequested) {
                // Extract pathName value from the auto JSON
                Pattern p = Pattern.compile("\"pathName\"\\s*:\\s*\"([^\"]+)\"");
                Matcher m = p.matcher(autoJson);
                if (m.find()) {
                  String pathName = m.group(1);
                  // Locate the path file in deploy/PathPlanner/paths
                  Path pathFile = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath().resolve("PathPlanner").resolve("paths").resolve(pathName + ".path");
                  if (Files.exists(pathFile)) {
                    String pathJson = Files.readString(pathFile, StandardCharsets.UTF_8);
                    // Find the first waypoint anchor x/y
                    Pattern wp = Pattern.compile("\"waypoints\"\\s*:\\s*\\[([\\s\\S]*?)\\]", Pattern.MULTILINE);
                    Matcher wpm = wp.matcher(pathJson);
                    if (wpm.find()) {
                      String waypointsArray = wpm.group(1);
                      Pattern anchor = Pattern.compile("\"anchor\"\\s*:\\s*\\{[^}]*?\"x\"\\s*:\\s*([0-9.+\\-Eed]+)\\s*,[^}]*?\"y\"\\s*:\\s*([0-9.+\\-Eed]+)");
                      Matcher am = anchor.matcher(waypointsArray);
                      if (am.find()) {
                        double x = Double.parseDouble(am.group(1));
                        double y = Double.parseDouble(am.group(2));
                        // Use 0 radians as a sensible default heading; PathPlanner may encode rotations elsewhere
                        Pose2d startPose = new Pose2d(x, y, new Rotation2d(0.0));
                        m_odometrySubsystem.resetOdometry(startPose);
                        Logger.recordOutput("Telemetry/Log", "[AutoChooser] Reset odometry to path start: " + startPose);
                      }
                    }
                  }
                }
              }
            }
          }
                } catch (IOException | RuntimeException e) {
                  // Non-fatal: log and fall back to origin reset
                  Logger.recordOutput("Telemetry/Errors", "[AutoChooser] Failed to parse selected auto for odometry reset: " + e.toString());
                  m_odometrySubsystem.resetOdometry(new Pose2d());
                }

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