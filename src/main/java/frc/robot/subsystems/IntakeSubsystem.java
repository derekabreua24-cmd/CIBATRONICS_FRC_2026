package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

/**
 * Intake / feeder: single SparkMax (brushed). <b>Brake mode required</b> when used as feeder
 * during shoot. Feeder can run in open-loop PID mode (voltage setpoint, no sensor feedback)
 * for consistent feed during shoot. All motor output is voltage (setVoltage); no percent.
 */
public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intake = new SparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushed);
  private final PIDController m_feederPid = new PIDController(
      IntakeConstants.kIntakeFeederP,
      IntakeConstants.kIntakeFeederI,
      IntakeConstants.kIntakeFeederD);
  private boolean m_reversed = false;
  /** Last commanded voltage (V) for telemetry. */
  private double m_lastCommandedVoltage = 0.0;
  /** Feeder open-loop PID setpoint (V). When non-zero, periodic() uses PID(0, setpoint) to output voltage. */
  private double m_feederVoltageSetpoint = 0.0;

  /** Brake mode; voltage comp at 12 V so setVoltage(x) actually outputs x volts. */
  public IntakeSubsystem() {
    m_feederPid.setIntegratorRange(-1.0, 1.0);
    com.revrobotics.spark.config.SparkMaxConfig cfg = new com.revrobotics.spark.config.SparkMaxConfig();
    cfg.inverted(false);
    cfg.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    cfg.voltageCompensation(12.0f); // Nominal bus so setVoltage(8) or setVoltage(12) is not capped
    m_intake.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }

  /** Runs intake at fixed voltage. When non-zero, magnitude is IntakeConstants.kIntakeVoltage. Reversed state flips sign. */
  public void runVoltage(double volts) {
    if (volts == 0.0) {
      m_intake.setVoltage(0.0);
      m_lastCommandedVoltage = 0.0;
      return;
    }
    double v = (m_reversed ? -1 : 1) * IntakeConstants.kIntakeVoltage * Math.signum(volts);
    m_intake.setVoltage(v);
    m_lastCommandedVoltage = v;
  }

  /** Convenience: run at kIntakeVoltage in given direction (speed sign: positive = forward, negative = reverse). */
  public void run(double speed) {
    runVoltage(speed == 0 ? 0 : Math.signum(speed) * IntakeConstants.kIntakeVoltage);
  }

  /** Run at a specific voltage (V). Sign is direction; magnitude applied directly (clamped to ±13 V). Used for unjam. */
  public void runAtVoltage(double volts) {
    m_feederVoltageSetpoint = 0.0; // leave feeder PID mode
    if (volts == 0.0) {
      m_intake.setVoltage(0.0);
      m_lastCommandedVoltage = 0.0;
      return;
    }
    double v = (m_reversed ? -1 : 1) * MathUtil.clamp(volts, -13.0, 13.0);
    m_intake.setVoltage(v);
    m_lastCommandedVoltage = v;
  }

  /** Set feeder voltage setpoint (V) for open-loop PID. Sign = direction; 0 = stop. Use during shoot. */
  public void setFeederVoltageSetpoint(double volts) {
    m_feederVoltageSetpoint = volts;
    if (volts == 0.0) {
      m_feederPid.reset();
      m_intake.setVoltage(0.0);
      m_lastCommandedVoltage = 0.0;
    }
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

  /** Detiene el motor del intake y clears feeder PID setpoint. */
  public void stop() {
    m_feederVoltageSetpoint = 0.0;
    m_feederPid.reset();
    m_intake.stopMotor();
    m_lastCommandedVoltage = 0.0;
  }

  @Override
  public void periodic() {
    if (m_feederVoltageSetpoint != 0.0) {
      double sign = Math.signum(m_feederVoltageSetpoint);
      double setpoint = Math.abs(m_feederVoltageSetpoint);
      double out = m_feederPid.calculate(0.0, setpoint);
      out = (m_reversed ? -sign : sign) * MathUtil.clamp(out, 0.0, IntakeConstants.kIntakeNominalVoltage);
      m_intake.setVoltage(out);
      m_lastCommandedVoltage = out;
    }
  }

  /** Last commanded voltage (V). All output is voltage. */
  public double getCommandedVoltage() {
    return m_lastCommandedVoltage;
  }

  /** Last applied output as fraction (-1..1). Prefer getCommandedVoltage() for voltage. */
  public double getSetpoint() {
    return m_intake.get();
  }

  /** Devuelve la corriente de salida (amperios) del motor del intake. No encoder (brushed motor). */
  public double getOutputCurrent() {
    return m_intake.getOutputCurrent();
  }
}
