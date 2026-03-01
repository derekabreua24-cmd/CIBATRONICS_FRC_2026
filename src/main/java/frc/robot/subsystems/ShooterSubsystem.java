package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_shooterFront;
  private final SparkMax m_shooterRear;
  @SuppressWarnings("removal")
  private final MotorControllerGroup m_shooterGroup;
  private final SimpleMotorFeedforward m_feedforward;
  private final PIDController m_pid;
  private double m_targetRpm = 0.0;
  // Guardar el ultimo porcentaje de salida calculado para registro
  private double m_lastOutputPercent = 0.0;

  @SuppressWarnings("removal")
public ShooterSubsystem() {

  // El shooter usa controladores SparkMax brushed (el tipo depende del motor conectado)
  m_shooterFront = new SparkMax(ShooterConstants.kShooterFrontMotorPort, MotorType.kBrushed);
  m_shooterRear = new SparkMax(ShooterConstants.kShooterRearMotorPort, MotorType.kBrushed);

  // Construir feedforward simple con las constantes definidas en ShooterConstants
  m_feedforward = new SimpleMotorFeedforward(
    ShooterConstants.kShooterKS,
    ShooterConstants.kShooterKV,
    ShooterConstants.kShooterKA);

  m_shooterGroup = new MotorControllerGroup(m_shooterFront, m_shooterRear);
  // Controlador PID (salida en porcentaje). Las ganancias se leeran desde la tabla "Tuning".
  m_pid = new PIDController(0.0, 0.0, 0.0);
  // Si el cableado requiere invertir un motor, usar setInverted en el grupo o configurar inversiones por motor.
  // Ejemplo: m_shooterGroup.setInverted(true);
  // Configuración segura para los motores del shooter.
  try {
    com.revrobotics.spark.config.SparkMaxConfig cfg = new com.revrobotics.spark.config.SparkMaxConfig();
    cfg.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
    cfg.smartCurrentLimit(60);
    cfg.openLoopRampRate(0.1);
    cfg.voltageCompensation(12.0f);
    m_shooterFront.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    m_shooterRear.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  } catch (RuntimeException e) {
  Logger.recordOutput("Shooter/Errors", "[ShooterSubsystem] SparkMax configure failed: " + e.toString());
  }
  }

  /** Establece la salida del shooter (-1..1). */
public void setSpeed(double speed) {
    m_shooterGroup.set(speed);
  }

  /** Establece el objetivo de velocidad del shooter (RPM). Usa feedforward y PID en periodic(). */
  public void setVelocitySetpointRpm(double targetRpm) {
    m_targetRpm = Math.abs(targetRpm);
  }

  /**
   * Establece la RPM del shooter según la distancia al objetivo (p. ej. desde visión o limelight).
   * Modelo lineal: RPM = base + pendiente × distancia, limitado a min/max. Afinar en ShooterConstants.ShooterDistanceConstants.
   */
  public void setVelocitySetpointFromDistanceMeters(double distanceMeters) {
    double rpm =
        ShooterConstants.ShooterDistanceConstants.kShooterRpmAt0M
            + ShooterConstants.ShooterDistanceConstants.kShooterRpmPerMeter
                * Math.max(0.0, distanceMeters);
    rpm =
        Math.max(
            ShooterConstants.ShooterDistanceConstants.kShooterRpmMin,
            Math.min(ShooterConstants.ShooterDistanceConstants.kShooterRpmMax, rpm));
    setVelocitySetpointRpm(rpm);
  }

  /** Detiene el shooter y limpia el setpoint para que periodic() no vuelva a comandar. */
  public void stop() {
    m_targetRpm = 0.0;
    m_shooterGroup.stopMotor();
  }

  public double getFrontVelocity() {
    return m_shooterFront.getEncoder().getVelocity();
  }

  public double getRearVelocity() {
    return m_shooterRear.getEncoder().getVelocity();
  }

  public double getAverageVelocity() {
    return (getFrontVelocity() + getRearVelocity()) / 2.0;
  }

  public double getOutputCurrent() {
    return m_shooterFront.getOutputCurrent() + m_shooterRear.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // Leer valores de afinacion desde la tabla Tuning y actualizar ganancias PID
    NetworkTable tuning = NetworkTableInstance.getDefault().getTable("Tuning");
    double p = tuning.getEntry("ShooterP").getDouble(m_pid.getP());
    double i = tuning.getEntry("ShooterI").getDouble(m_pid.getI());
    double d = tuning.getEntry("ShooterD").getDouble(m_pid.getD());
    if (p != m_pid.getP() || i != m_pid.getI() || d != m_pid.getD()) {
      m_pid.setPID(p, i, d);
    }

    // Si hay un RPM objetivo, calcular PID+FF y aplicar la salida; si no, detener motores
    if (m_targetRpm > 1.0) {
      double currentRpm = getAverageVelocity();
      double pidPercent = m_pid.calculate(currentRpm, m_targetRpm);

      double targetRadPerSec = m_targetRpm * 2.0 * Math.PI / 60.0;
      double ffVolts = m_feedforward.calculate(targetRadPerSec);
      double ffPercent = ffVolts / 12.0;

      double out = pidPercent + ffPercent;
      out = Math.max(-1.0, Math.min(1.0, out));
      m_shooterGroup.set(out);
      m_lastOutputPercent = out;
    } else {
      m_shooterGroup.set(0.0);
      m_lastOutputPercent = 0.0;
    }
  }

  /** Devuelve el objetivo de RPM actualmente solicitado. */
  public double getTargetRpm() {
    return m_targetRpm;
  }

  /** Devuelve el ultimo porcentaje de salida aplicado al grupo del shooter (-1..1). */
  public double getLastOutputPercent() {
    return m_lastOutputPercent;
  }
}
