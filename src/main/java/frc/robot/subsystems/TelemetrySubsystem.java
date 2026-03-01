package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.GeometryUtils;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/** Registra la telemetria del robot en la pestaña "Telemetry" de Shuffleboard. */
public class TelemetrySubsystem extends SubsystemBase {
  private final DriveSubsystem m_drive;
  private final NavXSubsystem m_navx;
  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;
  private final VisionSubsystem m_vision;
  private final CommandXboxController m_controller;
  private final CommandXboxController m_operatorController;

  private final GenericEntry m_leftAvgEntry;
  private final GenericEntry m_rightAvgEntry;
  private final GenericEntry m_intakeSetEntry;
  private final GenericEntry m_leftStickYEntry;
  private final GenericEntry m_rightStickXEntry;
  private final GenericEntry m_operatorLeftYEntry;
  private final GenericEntry m_operatorRightXEntry;

  // Telemetria expandida
  private final GenericEntry m_batteryVoltageEntry;
  private final GenericEntry m_enabledEntry;
  private final GenericEntry m_autonomousEntry;
  private final GenericEntry m_matchTimeEntry;
  private final GenericEntry m_fpgaTimeEntry;
  
  // Planificador / estadisticas de comandos
  // Comando activo por subsistema
  private final GenericEntry m_driveActiveCommandEntry;
  private final GenericEntry m_intakeActiveCommandEntry;
  // Subsistema de ejemplo eliminado — sin telemetria relacionada con el ejemplo
    // Driver tab compact entries
    private final GenericEntry m_driverLeftAvgEntry;
    private final GenericEntry m_driverShooterVelEntry;
    private final GenericEntry m_driverNavXYawEntry;
    private final GenericEntry m_driverMatchTimeEntry;
    private final GenericEntry m_driverEventLogEntry;
  private final GenericEntry m_shooterActiveCommandEntry;

  // Telemetria de codificadores/corriente del drive
  private final GenericEntry m_leftPosEntry;
    private final Field2d m_field2dDriver;
  private final GenericEntry m_leftVelEntry;
  private final GenericEntry m_leftCurrentEntry;
  private final GenericEntry m_leftFrontVoltEntry;
  private final GenericEntry m_leftRearVoltEntry;
  private final GenericEntry m_leftFrontTempEntry;
  private final GenericEntry m_leftRearTempEntry;
  private final GenericEntry m_rightPosEntry;
  private final GenericEntry m_rightVelEntry;
  private final GenericEntry m_rightCurrentEntry;
  private final GenericEntry m_rightFrontVoltEntry;
  private final GenericEntry m_rightRearVoltEntry;
  private final GenericEntry m_rightFrontTempEntry;
  private final GenericEntry m_rightRearTempEntry;

  // Telemetria del intake
  private final GenericEntry m_intakePosEntry;
  private final GenericEntry m_intakeVelEntry;
  private final GenericEntry m_intakeCurrentEntry;
  
  // Indexer removed from project — telemetry fields omitted
  private final GenericEntry m_shooterVelEntry;
  private final GenericEntry m_shooterCurrentEntry;
  private final GenericEntry m_navxYawEntry;
  private final GenericEntry m_navxPitchEntry;
  private final GenericEntry m_navxRollEntry;
  // Entradas de afinacion (PID y estimador)
  private final GenericEntry m_turnPEntry;
  private final GenericEntry m_turnIEntry;
  private final GenericEntry m_turnDEntry;
  private final GenericEntry m_turnToleranceEntry;
  private final GenericEntry m_estStateXEntry;
  private final GenericEntry m_estVisionXEntry;
  private final GenericEntry m_eventLogEntry;

  // Field2d para visualizar pose del robot y objetos de vision en Shuffleboard
  private final Field2d m_field2d;
  // Driver/Dev mode toggle
  private final GenericEntry m_driverModeEntry;

  // Snapshot logging (throttled) - runtime configurable
  private double m_snapshotPeriodSec = 0.2; // 5 Hz default
  private double m_lastSnapshotTime = 0.0;
  // Runtime display / logging controls
  private final GenericEntry m_extendedLoggingEntry;
  private final GenericEntry m_snapshotRateEntry;
  private final GenericEntry m_intakeReversedEntry;
  private final GenericEntry m_visionEnabledEntry;

  // Shooter and feed tuning entries (synced to Tuning table in periodic() for runtime tuning)
  private final GenericEntry m_shooterSetEntry;
  private final GenericEntry m_shooterPEntry;
  private final GenericEntry m_shooterIEntry;
  private final GenericEntry m_shooterDEntry;
  private final GenericEntry m_feedPulseEntry;
  private final GenericEntry m_feedPauseEntry;
  private final GenericEntry m_feedContinuousEntry;

  public TelemetrySubsystem(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, CommandXboxController controller, CommandXboxController operatorController, NavXSubsystem navx, VisionSubsystem vision) {
    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
    m_controller = controller;
    m_operatorController = operatorController;
    m_navx = navx;
  m_vision = vision;

    var driverTab = Shuffleboard.getTab("Telemetry Driver");
    var devTab = Shuffleboard.getTab("Telemetry Dev");
    // Usar devTab como 'tab' para la version detallada existente
    var tab = devTab;
    // Inicializar Field2d y añadirlo a Shuffleboard para visualizacion
    m_field2d = new Field2d();
    tab.add("Field", m_field2d).withSize(6, 4).withPosition(6, 0);
    // Field en la pestaña driver (compacta)
    m_field2dDriver = new Field2d();
    driverTab.add("Field", m_field2dDriver).withSize(6, 4).withPosition(6, 0);

    // Inicializar entradas compactas para la pestaña Driver
  m_driverLeftAvgEntry = driverTab.add("Drive Left Avg", 0.0).getEntry();
  m_driverShooterVelEntry = driverTab.add("Shooter Vel", 0.0).getEntry();
  m_driverNavXYawEntry = driverTab.add("NavX Yaw", 0.0).getEntry();
  m_driverMatchTimeEntry = driverTab.add("Match Time", 0.0).getEntry();
  m_driverEventLogEntry = driverTab.add("EventLog", "").withSize(3, 1).getEntry();

    // Si tenemos layout de AprilTags, dibujar todas las etiquetas conocidas en el Field2d
    if (m_vision != null) {
      var maybeLayout = m_vision.getFieldLayout();
      if (maybeLayout.isPresent()) {
        var layout = maybeLayout.get();
        // escanear IDs (0..99) y añadir las etiquetas encontradas
        for (int id = 0; id < 100; ++id) {
          var maybeTag = layout.getTagPose(id);
          if (maybeTag.isPresent()) {
            var p3 = maybeTag.get();
            var p2 = p3.toPose2d();
            try {
              m_field2d.getObject("Tag" + id).setPose(p2);
            } catch (RuntimeException e) {
              Logger.recordOutput("Telemetry/Errors", "TelemetrySubsystem: failed to set Field2d tag pose for id=" + id + " -> " + e.toString());
            }
          }
        }
      }
    }

  // Campos expandidos
  m_batteryVoltageEntry = tab.add("Battery Voltage", 0.0).getEntry();
  m_enabledEntry = tab.add("Enabled", false).getEntry();
  m_autonomousEntry = tab.add("Autonomous", false).getEntry();
  m_matchTimeEntry = tab.add("Match Time", 0.0).getEntry();
  m_fpgaTimeEntry = tab.add("FPGA Time", 0.0).getEntry();

  // Diseño de comandos activos (por subsistema)
  var activeLayout = tab.getLayout("Active Commands", BuiltInLayouts.kList).withSize(3, 2);
  m_driveActiveCommandEntry = activeLayout.add("Drive Active", "").getEntry();
  // Example active command removed
  m_intakeActiveCommandEntry = activeLayout.add("Intake Active", "").getEntry();
  m_shooterActiveCommandEntry = activeLayout.add("Shooter Active", "").getEntry();
  

  // Diseño de Drive - separar Left/Right para mejor lectura
  var leftDriveLayout = tab.getLayout("Drive Left", BuiltInLayouts.kList).withSize(3, 4).withPosition(0, 0);
  m_leftAvgEntry = leftDriveLayout.add("Drive Left Avg", 0.0).getEntry();
  m_leftPosEntry = leftDriveLayout.add("Left Pos", 0.0).getEntry();
  m_leftVelEntry = leftDriveLayout.add("Left Vel", 0.0).getEntry();
  m_leftCurrentEntry = leftDriveLayout.add("Left Current", 0.0).getEntry();
  m_leftFrontVoltEntry = leftDriveLayout.add("Left Front Volt", 0.0).getEntry();
  m_leftRearVoltEntry = leftDriveLayout.add("Left Rear Volt", 0.0).getEntry();
  m_leftFrontTempEntry = leftDriveLayout.add("Left Front Temp", 0.0).getEntry();
  m_leftRearTempEntry = leftDriveLayout.add("Left Rear Temp", 0.0).getEntry();

  var rightDriveLayout = tab.getLayout("Drive Right", BuiltInLayouts.kList).withSize(3, 4).withPosition(3, 0);
  m_rightAvgEntry = rightDriveLayout.add("Drive Right Avg", 0.0).getEntry();
  m_rightPosEntry = rightDriveLayout.add("Right Pos", 0.0).getEntry();
  m_rightVelEntry = rightDriveLayout.add("Right Vel", 0.0).getEntry();
  m_rightCurrentEntry = rightDriveLayout.add("Right Current", 0.0).getEntry();
  m_rightFrontVoltEntry = rightDriveLayout.add("Right Front Volt", 0.0).getEntry();
  m_rightRearVoltEntry = rightDriveLayout.add("Right Rear Volt", 0.0).getEntry();
  m_rightFrontTempEntry = rightDriveLayout.add("Right Front Temp", 0.0).getEntry();
  m_rightRearTempEntry = rightDriveLayout.add("Right Rear Temp", 0.0).getEntry();

  // Diseño de Intake
  var intakeLayout = tab.getLayout("Intake", BuiltInLayouts.kList).withSize(2, 3);
  m_intakeSetEntry = intakeLayout.add("Intake Setpoint", 0.0).getEntry();
  m_intakePosEntry = intakeLayout.add("Intake Pos", 0.0).getEntry();
  m_intakeVelEntry = intakeLayout.add("Intake Vel", 0.0).getEntry();
  m_intakeCurrentEntry = intakeLayout.add("Intake Current", 0.0).getEntry();
  m_intakeReversedEntry = intakeLayout.add("Intake Reversed", false).getEntry();

  // Indexer telemetry removed

  // Diseño de Shooter
  var shooterLayout = tab.getLayout("Shooter", BuiltInLayouts.kList).withSize(2, 3);
  m_shooterVelEntry = shooterLayout.add("Shooter Vel", 0.0).getEntry();
  m_shooterCurrentEntry = shooterLayout.add("Shooter Current", 0.0).getEntry();

  // Diseño de NavX
  var navxLayout = tab.getLayout("NavX", BuiltInLayouts.kList).withSize(2, 2);
  m_navxYawEntry = navxLayout.add("NavX Yaw", 0.0).getEntry();
  m_navxPitchEntry = navxLayout.add("NavX Pitch", 0.0).getEntry();
  m_navxRollEntry = navxLayout.add("NavX Roll", 0.0).getEntry();

  // Diseno de afinacion
  var tuningLayout = tab.getLayout("Tuning", BuiltInLayouts.kList).withSize(3, 3);
  m_turnPEntry = tuningLayout.add("Turn P", 0.02).getEntry();
  m_turnIEntry = tuningLayout.add("Turn I", 0.0).getEntry();
  m_turnDEntry = tuningLayout.add("Turn D", 0.001).getEntry();
  m_turnToleranceEntry = tuningLayout.add("Turn Tol Deg", 2.0).getEntry();
  // Vision fusion toggle (runtime A/B testing)
  m_visionEnabledEntry = tuningLayout.add("Vision Fusion Enabled", true).getEntry();

  m_estStateXEntry = tuningLayout.add("Estimator State Std X", 0.05).getEntry();
  m_estVisionXEntry = tuningLayout.add("Estimator Vision Std X", 0.5).getEntry();

  // Afinacion del shooter (expuesta para ajuste en vivo)
  var shooterTuningLayout = tab.getLayout("Shooter Tuning", BuiltInLayouts.kList).withSize(3, 2);
  m_shooterSetEntry = shooterTuningLayout.add("Shooter Setpoint RPM", 4500.0).getEntry();
  m_shooterPEntry = shooterTuningLayout.add("Shooter P", 0.0002).getEntry();
  m_shooterIEntry = shooterTuningLayout.add("Shooter I", 0.0).getEntry();
  m_shooterDEntry = shooterTuningLayout.add("Shooter D", 0.0).getEntry();
  m_feedPulseEntry = shooterTuningLayout.add("Feed Pulse Sec", 0.5).getEntry();
  m_feedPauseEntry = shooterTuningLayout.add("Feed Pause Sec", 0.2).getEntry();
  m_feedContinuousEntry = shooterTuningLayout.add("Feed Continuous", true).getEntry();

  // Registro de eventos (el ultimo evento se muestra en Shuffleboard y se escribe en DataLog)
  m_eventLogEntry = tab.add("EventLog", "").withSize(3, 1).getEntry();

  // Reflejar valores de afinacion en una NetworkTable simple para que los comandos los lean directamente.
  NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning");
  tuningTable.getEntry("TurnP").setDouble(m_turnPEntry.getDouble(0.02));
  tuningTable.getEntry("TurnI").setDouble(m_turnIEntry.getDouble(0.0));
  tuningTable.getEntry("TurnD").setDouble(m_turnDEntry.getDouble(0.001));
  tuningTable.getEntry("TurnTolDeg").setDouble(m_turnToleranceEntry.getDouble(2.0));
  tuningTable.getEntry("EstStateX").setDouble(m_estStateXEntry.getDouble(0.05));
  tuningTable.getEntry("EstVisionX").setDouble(m_estVisionXEntry.getDouble(0.5));

  // Initial sync to Tuning table (periodic() keeps it updated for runtime tuning)
  tuningTable.getEntry("ShooterSetpointRPM").setDouble(m_shooterSetEntry.getDouble(4500.0));
  tuningTable.getEntry("ShooterP").setDouble(m_shooterPEntry.getDouble(0.0002));
  tuningTable.getEntry("ShooterI").setDouble(m_shooterIEntry.getDouble(0.0));
  tuningTable.getEntry("ShooterD").setDouble(m_shooterDEntry.getDouble(0.0));
  tuningTable.getEntry("FeedPulseSec").setDouble(m_feedPulseEntry.getDouble(0.5));
  tuningTable.getEntry("FeedPauseSec").setDouble(m_feedPauseEntry.getDouble(0.2));
  tuningTable.getEntry("FeedContinuous").setBoolean(m_feedContinuousEntry.getBoolean(true));

  // Diseno de controles
  var controls = tab.getLayout("Controls", BuiltInLayouts.kList).withSize(2, 2).withPosition(0, 4);
  m_leftStickYEntry = controls.add("Left Stick Y", 0.0).getEntry();
  m_rightStickXEntry = controls.add("Right Stick X", 0.0).getEntry();
  m_operatorLeftYEntry = controls.add("Operator Left Y", 0.0).getEntry();
  m_operatorRightXEntry = controls.add("Operator Right X", 0.0).getEntry();

  // Display / runtime controls for telemetry
  var displayLayout = tab.getLayout("Display", BuiltInLayouts.kList).withSize(2, 2).withPosition(2, 4);
  m_extendedLoggingEntry = displayLayout.add("Extended Logging", true).getEntry();
  m_snapshotRateEntry = displayLayout.add("Snapshot Rate (Hz)", 5.0).getEntry();
  m_driverModeEntry = displayLayout.add("Driver Mode", false).getEntry();
  // Registrar eventos del ciclo de vida de comandos para trazabilidad
  CommandScheduler.getInstance().onCommandInitialize(cmd -> {
    String txt = "[Command] init: " + cmd.getClass().getSimpleName();
    try {
      Logger.recordOutput("Telemetry/CommandInit", txt);
    } catch (Throwable ignore) {}
    try {
      m_driverEventLogEntry.setString(txt);
    } catch (RuntimeException e) {
      Logger.recordOutput("Telemetry/Errors", "TelemetrySubsystem: failed to update driver event entry on init -> " + e.toString());
    }
  });
  CommandScheduler.getInstance().onCommandFinish(cmd -> {
    String txt = "[Command] finish: " + cmd.getClass().getSimpleName();
    try {
      Logger.recordOutput("Telemetry/CommandFinish", txt);
    } catch (Throwable ignore) {}
    try {
      m_driverEventLogEntry.setString(txt);
    } catch (RuntimeException e) {
      Logger.recordOutput("Telemetry/Errors", "TelemetrySubsystem: failed to update driver event entry on finish -> " + e.toString());
    }
  });
  CommandScheduler.getInstance().onCommandInterrupt(cmd -> {
    String txt = "[Command] interrupt: " + cmd.getClass().getSimpleName();
    try {
      Logger.recordOutput("Telemetry/CommandInterrupt", txt);
    } catch (Throwable ignore) {}
    try {
      m_driverEventLogEntry.setString(txt);
    } catch (RuntimeException e) {
      Logger.recordOutput("Telemetry/Errors", "TelemetrySubsystem: failed to update driver event entry on interrupt -> " + e.toString());
    }
  });
  }

  /** Registra un evento legible por humanos en Shuffleboard y en el registro de datos. */
  public void logEvent(String event) {
    String timestamped = String.format("%.3f: %s", Timer.getFPGATimestamp(), event);
    // Mostrar el ultimo evento en Shuffleboard
    m_eventLogEntry.setString(timestamped);
    // Also update the compact driver event log entry if present
  try { m_driverEventLogEntry.setString(timestamped); } catch (RuntimeException e) { Logger.recordOutput("Telemetry/Errors", "TelemetrySubsystem: failed to set driver event entry -> " + e.toString()); }
    // Tambien escribir en el registro persistente de datos (AdvantageKit Logger)
  Logger.recordOutput("Telemetry/Event", timestamped);
  }

  @Override
  public void periodic() {
  // Odometry is updated only in OdometrySubsystem.periodic() to avoid double-updating the pose estimator.

  // Sync Shuffleboard tuning entries to Tuning table so ShooterSubsystem and ShootSequenceCommand see runtime changes
  NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning");
  tuningTable.getEntry("TurnP").setDouble(m_turnPEntry.getDouble(0.02));
  tuningTable.getEntry("TurnI").setDouble(m_turnIEntry.getDouble(0.0));
  tuningTable.getEntry("TurnD").setDouble(m_turnDEntry.getDouble(0.001));
  tuningTable.getEntry("TurnTolDeg").setDouble(m_turnToleranceEntry.getDouble(2.0));
  tuningTable.getEntry("EstStateX").setDouble(m_estStateXEntry.getDouble(0.05));
  tuningTable.getEntry("EstVisionX").setDouble(m_estVisionXEntry.getDouble(0.5));
  tuningTable.getEntry("ShooterSetpointRPM").setDouble(m_shooterSetEntry.getDouble(4500.0));
  tuningTable.getEntry("ShooterP").setDouble(m_shooterPEntry.getDouble(0.0002));
  tuningTable.getEntry("ShooterI").setDouble(m_shooterIEntry.getDouble(0.0));
  tuningTable.getEntry("ShooterD").setDouble(m_shooterDEntry.getDouble(0.0));
  tuningTable.getEntry("FeedPulseSec").setDouble(m_feedPulseEntry.getDouble(0.5));
  tuningTable.getEntry("FeedPauseSec").setDouble(m_feedPauseEntry.getDouble(0.2));
  tuningTable.getEntry("FeedContinuous").setBoolean(m_feedContinuousEntry.getBoolean(true));

  // Si la vision tiene una pose reciente, pasarla al estimador para fusion con marca de tiempo
    if (m_vision != null) {
  var maybePose = m_vision.getLastPose();
  var maybeTs = m_vision.getLastTimestamp();
        if (maybePose.isPresent() && maybeTs.isPresent()) {
        // Only apply vision measurements when not in teleop and when the runtime toggle
        // for vision fusion is enabled. This lets us do A/B testing at runtime.
        boolean visionEnabled = true;
        try {
          visionEnabled = m_visionEnabledEntry.getBoolean(true);
        } catch (RuntimeException ex) {
          // If Shuffleboard entry missing, default to enabled
          visionEnabled = true;
        }
        if (visionEnabled && !edu.wpi.first.wpilibj.DriverStation.isTeleop()) {
          m_drive.addVisionMeasurement(maybePose.get(), maybeTs.getAsDouble());
        }
      }
    }

  // Modo Driver vs Dev: cuando Driver Mode está activado, actualizamos solo los widgets compactos
  boolean driverMode = m_driverModeEntry.getBoolean(false);

    // Actualizar las entradas de Shuffleboard con los valores actuales (segun el modo seleccionado)
  // Las entradas de promedio left/right se establecen aqui si no se agregaron antes al layout
  // (mantiene compatibilidad con rutas de codigo de layout anteriores).
  if (driverMode) {
    // Actualizar solo widgets compactos en la pestaña Driver
    m_driverLeftAvgEntry.setDouble(m_drive.getLeftAverage());
    m_driverShooterVelEntry.setDouble(m_shooter.getAverageVelocity());
    m_driverNavXYawEntry.setDouble(m_navx.getYaw());
    m_driverMatchTimeEntry.setDouble(DriverStation.getMatchTime());
  } else {
    m_leftAvgEntry.setDouble(m_drive.getLeftAverage());
    m_rightAvgEntry.setDouble(m_drive.getRightAverage());
    m_intakeSetEntry.setDouble(m_intake.getSetpoint());
  }

  // Ejes del controlador
  m_leftStickYEntry.setDouble(m_controller.getLeftY());
  m_rightStickXEntry.setDouble(m_controller.getRightX());
  // Ejes del controlador operador (si esta disponible)
    if (m_operatorController != null) {
      m_operatorLeftYEntry.setDouble(m_operatorController.getLeftY());
      m_operatorRightXEntry.setDouble(m_operatorController.getRightX());
    }

  // Update intake reversed indicator
  try {
    m_intakeReversedEntry.setBoolean(m_intake.isReversed());
  } catch (RuntimeException e) {
    Logger.recordOutput("Telemetry/Errors", "TelemetrySubsystem: failed to set intake reversed entry -> " + e.toString());
  }

    // Telemetria expandida desde WPILib
    m_batteryVoltageEntry.setDouble(RobotController.getBatteryVoltage());
  // Tambien escribir valores importantes en el registro persistente (AdvantageKit Logger)
  Logger.recordOutput("Telemetry/BatteryVoltage", RobotController.getBatteryVoltage());
    m_enabledEntry.setBoolean(DriverStation.isEnabled());
    m_autonomousEntry.setBoolean(DriverStation.isAutonomous());
    m_matchTimeEntry.setDouble(DriverStation.getMatchTime());
    m_fpgaTimeEntry.setDouble(Timer.getFPGATimestamp());

  // Registrar promedios del drive y setpoints del intake periodicamente
  Logger.recordOutput("Telemetry/DriveLeftAvg", m_drive.getLeftAverage());
  Logger.recordOutput("Telemetry/DriveRightAvg", m_drive.getRightAverage());
  Logger.recordOutput("Telemetry/IntakeSet", m_intake.getSetpoint());

  // Drive/intake encoder & current telemetry (dev only)
  boolean extended = m_extendedLoggingEntry.getBoolean(true);
  if (!driverMode) {
    m_leftPosEntry.setDouble(m_drive.getLeftAveragePosition());
    m_leftVelEntry.setDouble(m_drive.getLeftAverageVelocity());
    m_leftCurrentEntry.setDouble(m_drive.getLeftTotalCurrent());

    m_rightPosEntry.setDouble(m_drive.getRightAveragePosition());
    m_rightVelEntry.setDouble(m_drive.getRightAverageVelocity());
    m_rightCurrentEntry.setDouble(m_drive.getRightTotalCurrent());

  // Update optional detailed per-motor telemetry only when extended logging enabled
  if (extended) {
      m_leftFrontVoltEntry.setDouble(m_drive.getLeftFrontVoltage());
      m_leftRearVoltEntry.setDouble(m_drive.getLeftRearVoltage());
      m_leftFrontTempEntry.setDouble(m_drive.getLeftFrontTemperature());
      m_leftRearTempEntry.setDouble(m_drive.getLeftRearTemperature());
      m_rightFrontVoltEntry.setDouble(m_drive.getRightFrontVoltage());
      m_rightRearVoltEntry.setDouble(m_drive.getRightRearVoltage());
      m_rightFrontTempEntry.setDouble(m_drive.getRightFrontTemperature());
      m_rightRearTempEntry.setDouble(m_drive.getRightRearTemperature());
    }
  } else {
    // In driver mode, update driver Field2d as well
    try {
      m_field2dDriver.setRobotPose(m_drive.getPose());
    } catch (RuntimeException e) {
      Logger.recordOutput("Telemetry/Errors", "TelemetrySubsystem: failed to set driver Field2d pose -> " + e.toString());
    }
  }

    m_intakePosEntry.setDouble(m_intake.getEncoderPosition());
    m_intakeVelEntry.setDouble(m_intake.getEncoderVelocity());
    m_intakeCurrentEntry.setDouble(m_intake.getOutputCurrent());

  // Telemetria NavX
  m_navxYawEntry.setDouble(m_navx.getYaw());
  m_navxPitchEntry.setDouble(m_navx.getPitch());
  m_navxRollEntry.setDouble(m_navx.getRoll());


  // Telemetria del shooter
  m_shooterVelEntry.setDouble(m_shooter.getAverageVelocity());
  m_shooterCurrentEntry.setDouble(m_shooter.getOutputCurrent());

  // Nombres de comandos activos por subsistema
    Command dcmd = m_drive.getCurrentCommand();
    m_driveActiveCommandEntry.setString(dcmd == null ? "None" : dcmd.getClass().getSimpleName());
    Command icmd = m_intake.getCurrentCommand();
    m_intakeActiveCommandEntry.setString(icmd == null ? "None" : icmd.getClass().getSimpleName());
    Command scmd = m_shooter.getCurrentCommand();
    m_shooterActiveCommandEntry.setString(scmd == null ? "None" : scmd.getClass().getSimpleName());

    // Throttled comprehensive snapshot log (CSV-ish) to DataLogManager
    // Allow runtime tuning of snapshot rate
    double reqHz = m_snapshotRateEntry.getDouble(5.0);
    if (reqHz > 0.001) {
      m_snapshotPeriodSec = 1.0 / reqHz;
    }

    double now = Timer.getFPGATimestamp();
    if (now - m_lastSnapshotTime >= m_snapshotPeriodSec) {
      m_lastSnapshotTime = now;

      Pose2d pose = m_drive.getPose();
      // Actualizar Field2d con la pose estimada del robot
      try {
        m_field2d.setRobotPose(pose);
      } catch (RuntimeException e) {
        Logger.recordOutput("Telemetry/Errors", "TelemetrySubsystem: failed to set Field2d robot pose -> " + e.toString());
      }
      String visionX = "";
      String visionY = "";
      String visionDeg = "";
      String visionTs = "";
      if (m_vision != null) {
        var maybePose = m_vision.getLastPose();
        var maybeTs = m_vision.getLastTimestamp();
        if (maybePose.isPresent()) {
          Pose2d vp = maybePose.get();
          // Mostrar la pose de vision en Field2d
          try {
            m_field2d.getObject("Vision").setPose(vp);
          } catch (RuntimeException e) {
            Logger.recordOutput("Telemetry/Errors", "TelemetrySubsystem: failed to set Field2d vision pose -> " + e.toString());
          }
          visionX = String.format("%.3f", vp.getX());
          visionY = String.format("%.3f", vp.getY());
          visionDeg = String.format("%.2f", vp.getRotation().getDegrees());

          // ---- New: publish a small Pose2d[] of vision candidates using Transform2d ----
          try {
            Pose2d base = vp;
            Pose2d[] candidates = new Pose2d[] {
              base,
              base.transformBy(new Transform2d(new Translation2d(0.2, 0.0), new Rotation2d(0.0))),
              base.transformBy(new Transform2d(new Translation2d(-0.2, 0.0), new Rotation2d(0.0)))
            };
            GeometryUtils.publishPose2dArrayToField2d(m_field2d, "VisionCandidates", candidates);
          } catch (RuntimeException e) {
            Logger.recordOutput("Telemetry/Errors", "TelemetrySubsystem: failed to publish Pose2d[] candidates -> " + e.toString());
          }
        }
        if (maybeTs.isPresent()) {
          visionTs = String.format("%.3f", maybeTs.getAsDouble());
        }
      }

  // Compact vs extended snapshot (compact UI mode is handled separately)
      java.util.List<String> parts = new java.util.ArrayList<>();
      parts.add(String.format("%.3f", now));
      parts.add(String.format("%.3f", DriverStation.getMatchTime()));
      parts.add(String.format("%.3f", now)); // fpgaTime
      parts.add(String.format("%.3f", pose.getX()));
      parts.add(String.format("%.3f", pose.getY()));
      parts.add(String.format("%.2f", pose.getRotation().getDegrees()));
      parts.add(visionX);
      parts.add(visionY);
      parts.add(visionDeg);
      parts.add(visionTs);

      // Drive basics
      parts.add(String.format("%.3f", m_drive.getLeftAveragePosition()));
      parts.add(String.format("%.3f", m_drive.getLeftAverageVelocity()));
      parts.add(String.format("%.3f", m_drive.getLeftTotalCurrent()));

      if (extended) {
  // Campos detallados del motor izquierdo
        parts.add(String.format("%.3f", m_drive.getLeftFrontCurrent()));
        parts.add(String.format("%.3f", m_drive.getLeftRearCurrent()));
        parts.add(String.format("%.3f", m_drive.getLeftFrontOutput()));
        parts.add(String.format("%.3f", m_drive.getLeftRearOutput()));
        parts.add(String.format("%.3f", m_drive.getLeftFrontVoltage()));
        parts.add(String.format("%.3f", m_drive.getLeftFrontTemperature()));
        parts.add(String.format("%.3f", m_drive.getLeftRearVoltage()));
        parts.add(String.format("%.3f", m_drive.getLeftRearTemperature()));
      }

      parts.add(String.format("%.3f", m_drive.getLeftAverage()));
      parts.add(String.format("%.3f", m_drive.getRightAveragePosition()));
      parts.add(String.format("%.3f", m_drive.getRightAverageVelocity()));
      parts.add(String.format("%.3f", m_drive.getRightTotalCurrent()));

      if (extended) {
        parts.add(String.format("%.3f", m_drive.getRightFrontCurrent()));
        parts.add(String.format("%.3f", m_drive.getRightRearCurrent()));
        parts.add(String.format("%.3f", m_drive.getRightFrontOutput()));
        parts.add(String.format("%.3f", m_drive.getRightRearOutput()));
        parts.add(String.format("%.3f", m_drive.getRightFrontVoltage()));
        parts.add(String.format("%.3f", m_drive.getRightFrontTemperature()));
        parts.add(String.format("%.3f", m_drive.getRightRearVoltage()));
        parts.add(String.format("%.3f", m_drive.getRightRearTemperature()));
      }

      parts.add(String.format("%.3f", m_drive.getRightAverage()));

  // Intake/shooter
      parts.add(String.format("%.3f", m_intake.getSetpoint()));
      parts.add(String.format("%.3f", m_intake.getEncoderPosition()));
      parts.add(String.format("%.3f", m_intake.getEncoderVelocity()));
      parts.add(String.format("%.3f", m_intake.getOutputCurrent()));


      parts.add(String.format("%.3f", m_shooter.getTargetRpm()));
      parts.add(String.format("%.3f", m_shooter.getAverageVelocity()));
      parts.add(String.format("%.3f", m_shooter.getOutputCurrent()));
      parts.add(String.format("%.3f", m_shooter.getLastOutputPercent()));

      // NavX and controls
      parts.add(String.format("%.3f", m_navx.getYaw()));
      parts.add(String.format("%.3f", m_navx.getPitch()));
      parts.add(String.format("%.3f", m_navx.getRoll()));
      parts.add(String.format("%.3f", m_controller.getLeftY()));
      parts.add(String.format("%.3f", m_controller.getRightX()));
      parts.add(m_operatorController == null ? "" : String.format("%.3f", m_operatorController.getLeftY()));
      parts.add(m_operatorController == null ? "" : String.format("%.3f", m_operatorController.getRightX()));

      parts.add(dcmd == null ? "None" : dcmd.getClass().getSimpleName());
      parts.add(icmd == null ? "None" : icmd.getClass().getSimpleName());
      parts.add(scmd == null ? "None" : scmd.getClass().getSimpleName());

      String csv = String.join(",", parts);
  Logger.recordOutput("Telemetry/Snapshot", csv);
    }
  }
}
