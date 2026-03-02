package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

/** Subsistema sencillo de intake usando un único SparkMax. Use kBrushless para NEO; kBrushed para brushed (ver IntakeConstants). */
public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intake = new SparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushed);
  // Si es true, el sentido del motor del intake está invertido (run(...) cambia el signo).
  private boolean m_reversed = false;

  /** 2026 API: inversión por config en lugar de setInverted(). */
  public IntakeSubsystem() {
    com.revrobotics.spark.config.SparkMaxConfig cfg = new com.revrobotics.spark.config.SparkMaxConfig();
    cfg.inverted(false);
    m_intake.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }

  /** Runs the intake motor at the given output magnitude (use IntakeConstants.kDefaultSpeed). */
  public void run(double speed) {
    m_intake.set(m_reversed ? -speed : speed);
  }

  /** Alterna el sentido del intake; las llamadas a run() posteriores se invertirán cuando esté en reversa. */
  public void toggleReverse() {
    m_reversed = !m_reversed;
    org.littletonrobotics.junction.Logger.recordOutput("Intake/State", "Reversed=" + m_reversed);
  }

  /** Devuelve true si el sentido del intake está actualmente invertido. */
  public boolean isReversed() {
    return m_reversed;
  }

  /** Detiene el motor del intake. */
  public void stop() {
    m_intake.stopMotor();
  }

  @Override
  public void periodic() {
    // Nada por ahora
  }

  /** Devuelve el último setpoint solicitado para el motor del intake (-1..1). */
  public double getSetpoint() {
    return m_intake.get();
  }

  /** Devuelve la posición del encoder (rotaciones) del motor del intake. */
  public double getEncoderPosition() {
    return m_intake.getEncoder().getPosition();
  }

  /** Devuelve la velocidad del encoder del motor del intake. */
  public double getEncoderVelocity() {
    return m_intake.getEncoder().getVelocity();
  }

  /** Devuelve la corriente de salida (amperios) del motor del intake. */
  public double getOutputCurrent() {
    return m_intake.getOutputCurrent();
  }
}
