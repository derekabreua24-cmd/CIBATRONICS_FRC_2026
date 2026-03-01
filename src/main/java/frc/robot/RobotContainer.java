package frc.robot;

import frc.robot.Camera_Calibration.CameraCalibrationLoader;
import frc.robot.Camera_Calibration.CameraCalibrationLoader.Calibration;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ShooterConstants;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Drv_Commands.DriveCommand;
import frc.robot.commands.Drv_Commands.TurnToAngleCommand;
// IndexerSubsystem eliminado; toda la funcionalidad del indexer fue recortada del proyecto.
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

public class RobotContainer {

  private final NavXSubsystem m_navxSubsystem = new NavXSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final OdometrySubsystem m_odometrySubsystem =
      new OdometrySubsystem(m_driveSubsystem, m_navxSubsystem);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TelemetrySubsystem m_telemetrySubsystem;

  private VisionSubsystem m_visionSubsystem = null;
  private UsbAprilTagProcessor m_usbProcessor = null;

    private edu.wpi.first.wpilibj.smartdashboard.SendableChooser<Command> m_ppAutoChooser = null;

  private GenericEntry m_targetXEntry;
  private GenericEntry m_targetYEntry;
  private GenericEntry m_targetHeadingEntry;
  private GenericEntry m_maxSpeedEntry;
  private GenericEntry m_posTolEntry;
  private GenericEntry m_angTolEntry;
    private GenericEntry m_ppltvGainEntry;
    private GenericEntry m_resetOdomEntry;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  // Entrada de ajuste de RPM del shooter (modificable en tiempo de ejecución).
  private GenericEntry m_shooterRpmEntry;

  public RobotContainer() {

    // Crear pestaña Tuning y entrada de RPM del shooter antes de configureBindings() para que ShooterCommand reciba una entrada válida.
    var tuningTab = Shuffleboard.getTab("Tuning");
    m_shooterRpmEntry = tuningTab.add("Shooter RPM", ShooterConstants.kShooterMaxRPM * Math.abs(ShooterConstants.kShooterSpeed)).withPosition(8, 0).withSize(2, 1).getEntry();

    try {

      Calibration calib =
          CameraCalibrationLoader.loadFromProperties(
              "camera/camera_calib.properties");

      // 1) Preferir layouts 2026 incorporados si existen (WPILib 2026.2+: k2026RebuiltAndymark, k2026RebuiltWelded). loadField() es la API actual (sustituye loadAprilTagLayoutField obsoleta).
      edu.wpi.first.apriltag.AprilTagFieldLayout fieldLayout = null;
      for (edu.wpi.first.apriltag.AprilTagFields f : edu.wpi.first.apriltag.AprilTagFields.values()) {
        if (f.name().toLowerCase().contains("2026")) {
          try {
            fieldLayout = edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(f);
            Logger.recordOutput("Telemetry/Log", "Using built-in AprilTagFieldLayout: " + f.name());
            break;
          } catch (Throwable t) {
            Logger.recordOutput("Telemetry/Errors", "AprilTagFieldLayout.loadField(" + f.name() + ") failed: " + t.toString());
          }
        }
      }

      try {
        if (fieldLayout == null) {
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
          // API actual: el constructor AprilTagFieldLayout(Path) carga desde JSON (sin reflexión).
          try {
            fieldLayout = new edu.wpi.first.apriltag.AprilTagFieldLayout(found);
            Logger.recordOutput("Telemetry/Log", "Layout de campo AprilTag cargado desde deploy: " + found.toString());
          } catch (java.io.IOException e) {
            Logger.recordOutput("Telemetry/Errors", "Error al cargar AprilTagFieldLayout desde archivo: " + e.toString());
          }
        }

        if (fieldLayout == null) {
          // Respaldo: recurso incorporado (puede ser de versión anterior).
          fieldLayout = edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
              edu.wpi.first.apriltag.AprilTagFields.kDefaultField);
          Logger.recordOutput("Telemetry/Log", "Usando AprilTagFieldLayout incorporado (predeterminado)");
        }
        else {
          // Se cargó desde deploy; opcionalmente exportar el layout a deploy para inspección/distribución.
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
        }
      } catch (Throwable t) {
        // Respaldo defensivo: si algo falla, intentar cargar el layout de campo predeterminado.
        try {
          fieldLayout = edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
              edu.wpi.first.apriltag.AprilTagFields.kDefaultField);
        } catch (Throwable t2) {
          Logger.recordOutput("Telemetry/Errors", "Failed to initialize AprilTagFieldLayout: " + t2.toString());
          fieldLayout = null;
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

    var autoTab = Shuffleboard.getTab("Autonomous");

  m_targetXEntry = autoTab.add("Target X (m)", 1.5).getEntry();
    m_targetYEntry = autoTab.add("Target Y (m)", 0.0).getEntry();
    m_targetHeadingEntry = autoTab.add("Target Heading (deg)", 0.0).getEntry();
    m_maxSpeedEntry = autoTab.add("Max Speed (m/s)", 0.6).getEntry();
    m_posTolEntry = autoTab.add("Pos Tol (m)", 0.1).getEntry();
    m_angTolEntry = autoTab.add("Ang Tol (deg)", 5.0).getEntry();

  // Opción para reiniciar la odometría al inicio del auto seleccionado (cuando el .auto tiene resetOdom=true).
  var resetOdomWidget = autoTab.add("Reiniciar odom al inicio de ruta", true).withPosition(0, 4).withSize(2, 1);
  m_resetOdomEntry = resetOdomWidget.getEntry();

  // PPLTVController dt (s): paso de discretización, típicamente 0.02 para bucle 50 Hz (API PathPlanner 2026).
    m_ppltvGainEntry = tuningTab.add("PPLTV dt (s)", 0.02).withPosition(0, 0).withSize(2, 1).getEntry();
  // Entradas de feedforward de la unidad de tracción (usadas por DriveSubsystem si están habilitadas)
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

    // ================= PATHPLANNER 2026.1.2 CONFIG =================
    try {

  RobotConfig config = RobotConfig.fromGUISettings();

      // PPLTVController(dt): dt = paso de discretización en segundos (0.02 = bucle FRC 20 ms). Ver API PathPlanner.
      double ppltvDt = m_ppltvGainEntry.getDouble(0.02);
      if (ppltvDt <= 0 || ppltvDt > 0.1) ppltvDt = 0.02;

      AutoBuilder.configure(
    m_odometrySubsystem::getPose,              // Proveedor de pose
    m_odometrySubsystem::resetOdometry,         // Reinicio de pose (PathPlanner llama solo con Pose2d)
    m_driveSubsystem::getChassisSpeeds,         // ChassisSpeeds relativos al robot
    m_driveSubsystem::driveWithSpeeds,
    new PPLTVController(ppltvDt),
    config,
    () -> DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red,
    m_driveSubsystem
);
  Logger.recordOutput("Telemetry/Log", "PathPlanner 2026 fully configured.");

      // Registrar ruta/objetivo en AdvantageKit para que AdvantageScope los muestre en la sección Poses del campo 2D/3D.
      PathPlannerLogging.setLogCurrentPoseCallback(
          pose -> Logger.recordOutput("PathPlanner/CurrentPose", pose));
      PathPlannerLogging.setLogTargetPoseCallback(
          pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));
      PathPlannerLogging.setLogActivePathCallback(
          poses -> Logger.recordOutput("PathPlanner/ActivePath",
              poses.toArray(new edu.wpi.first.math.geometry.Pose2d[0])));

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

                            // Hacer que sea la opción predeterminada en el selector.
                            m_ppAutoChooser.setDefaultOption("Ejemplo: recto (programático)", exampleCmd);
            } catch (RuntimeException e) {
              Logger.recordOutput("Telemetry/Errors", "PathPlanner chooser: failed to add example option -> " + e.toString());
            }
      } catch (RuntimeException e) {
  Logger.recordOutput("Telemetry/Errors", "Failed to build PathPlanner auto chooser: " + e.getMessage());
      }

    } catch (Exception e) {
      // RobotConfig.fromGUISettings puede lanzar excepciones de parseo del cargador de config PathPlanner;
      // captura amplia para no bloquear la inicialización del robot y registrar la causa.
  Logger.recordOutput("Telemetry/Errors", "Falló la configuración de PathPlanner: " + e.getMessage());
    }
    // ================================================================

    configureBindings();
    m_driveSubsystem.setDefaultCommand(
        new DriveCommand(m_driveSubsystem, m_driverController));
  }

  private void configureBindings() {
    // SysId: Left Bumper + face buttons para caracterización del tren de rodaje (ver CHARACTERIZATION.md).
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

    m_driverController.y()
        .onTrue(new LogEventCommand(
            m_telemetrySubsystem,
            "Manual event: Driver Y pressed"));

    m_operatorController.y()
        .onTrue(new LogEventCommand(
            m_telemetrySubsystem,
            "Manual event: Operator Y pressed"));

    // Detener intake y shooter de inmediato (emergencia / desbloquear).
    m_operatorController.b()
        .onTrue(new frc.robot.commands.StopMechanismsCommand(m_intakeSubsystem, m_shooterSubsystem));

    // Intake: gatillo izquierdo (sentido por defecto; A cambia sentido).
    m_operatorController.leftTrigger()
        .whileTrue(new IntakeCommand(m_intakeSubsystem));

    m_operatorController.a()
        .onTrue(new ToggleIntakeDirectionCommand(m_intakeSubsystem));

    // Unjam: un tap = pulso corto de reversa para desatascar (sin mantener).
    m_operatorController.leftBumper()
        .onTrue(new frc.robot.commands.UnjamCommand(m_intakeSubsystem));

    // Intake en reversa mientras se mantiene el botón.
    m_operatorController.rightBumper()
        .whileTrue(new frc.robot.commands.ReverseIntakeCommand(m_intakeSubsystem));

    // Shooter: gatillo derecho (RPM en pestaña Tuning). X = secuencia shooter+intake.
    m_operatorController.rightTrigger()
        .whileTrue(new ShooterCommand(m_shooterSubsystem, m_shooterRpmEntry, m_visionSubsystem));

    m_operatorController.x()
        .whileTrue(new frc.robot.commands.ShootSequenceCommand(m_shooterSubsystem, m_intakeSubsystem, m_visionSubsystem));

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

    
        // Si el selector de PathPlanner está disponible y tiene un comando seleccionado, usarlo.
        if (m_ppAutoChooser != null) {
            Command ppCmd = m_ppAutoChooser.getSelected();
            if (ppCmd != null) {
                // Reiniciar sensores/odometría antes de ejecutar el auto seleccionado.
                m_navxSubsystem.reset();
        // Intentar leer el nombre seleccionado en el selector desde NetworkTables de Shuffleboard.
                try {
          var table = NetworkTableInstance.getDefault()
            .getTable("Shuffleboard")
            .getSubTable("Autonomous")
            .getSubTable("PathPlanner Autos");

          String selected = table.getEntry("selected").getString("");
          if (selected == null || selected.isEmpty()) {
            // Algunas variantes de SendableChooser añaden prefijo; probar entrada por defecto.
            selected = table.getEntry("default").getString("");
          }

                    if (selected != null && !selected.isEmpty()) {
            // Deploy folder is "pathplanner" (lowercase); RoboRIO filesystem is case-sensitive.
            Path deployAutos = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos");
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
              // Solo hacer el reinicio automático de odometría si el usuario activó el toggle en Shuffleboard.
              boolean userRequested = m_resetOdomEntry.getBoolean(true);
              if (resetOdom && userRequested) {
                // Extract pathName value from the auto JSON
                Pattern p = Pattern.compile("\"pathName\"\\s*:\\s*\"([^\"]+)\"");
                Matcher m = p.matcher(autoJson);
                if (m.find()) {
                  String pathName = m.group(1);
                  Path pathFile = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("paths").resolve(pathName + ".path");
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
                        // Usar 0 rad como rumbo por defecto; PathPlanner puede codificar rotaciones en otro sitio.
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
                  // No fatal: registrar y usar reinicio al origen.
                  Logger.recordOutput("Telemetry/Errors", "[AutoChooser] Error al parsear el auto seleccionado para reinicio de odometría: " + e.toString());
                  m_odometrySubsystem.resetOdometry(new Pose2d());
                }

        return ppCmd;
      }
    }

    // No hay auto de PathPlanner seleccionado; devolver comando nulo.
    return Commands.none();
  }

  public void shutdownVision() {
    if (m_usbProcessor != null) {
      m_usbProcessor.stop();
    }
  }

  /**
   * Llamado cada ciclo de simulación. Inyecta pose/distance sintéticos en VisionSubsystem cuando no hay cámara,
   * para que AdvantageScope muestre Vision/RobotPose y Vision/TargetDistanceMeters y se pueda probar la tubería.
   */
  public void simulationPeriodic() {
    if (edu.wpi.first.wpilibj.RobotBase.isSimulation() && m_visionSubsystem != null) {
      Pose2d pose = m_odometrySubsystem.getPose();
      m_visionSubsystem.setSimulationPoseAndDistance(pose, 2.0);
    }
  }
}