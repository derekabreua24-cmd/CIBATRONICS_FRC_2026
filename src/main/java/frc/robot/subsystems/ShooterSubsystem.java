package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.ShooterConstants;

/**
 * Shooter (NEO) and feed-mode configuration.
 *
 * <p><b>Brake mode:</b> NEO is configured IdleMode.kBrake so it stops when the command ends;
 * the feeder motor (intake) must also be in Brake mode (see IntakeSubsystem).
 *
 * <p><b>Velocity PID:</b> Shooter uses closed-loop velocity (PID + feedforward) via
 * setVelocitySetpointRpm(). Constant RPM compensates for speed drop when a ball hits the wheels,
 * preventing the second ball from catching the first (jams/double shots).
 *
 * <p><b>Feeder gating:</b> The shoot command must only run the feeder after (1) shooter reaches
 * at least 95% of target RPM (velocity check) and (2) a time-based delay (e.g. 0.65 s). Feeder
 * speed should be 30–40% to keep flow controlled.
 */
public class ShooterSubsystem extends SubsystemBase {
  private static final double kNominalVoltage = ShooterConstants.kShooterVoltage;

  private final SparkMax m_shooter;
  private final PIDController m_pid = new PIDController(
      ShooterConstants.kShooterP,
      ShooterConstants.kShooterI,
      ShooterConstants.kShooterD);
  private final SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(
      ShooterConstants.kShooterKS,
      ShooterConstants.kShooterKV,
      ShooterConstants.kShooterKA);
  private double m_targetRpm = 0.0;
  private double m_lastTargetRpm = -1.0;
  /** When > 0, motor runs at this voltage (V) to feed during intake. Set via setFeedVoltage(volts). */
  private double m_feedVoltage = 0.0;
  /** When > 0, motor runs at this voltage (V) in shooting direction (open-loop). Used by ShooterCommand for fixed 11 V shoot. */
  private double m_shootVoltage = 0.0;
  private double m_lastOutputPercent = 0.0;

  public ShooterSubsystem() {
    m_shooter = new SparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless); // NEO: brushless, internal encoder
    // WPILib: setIntegratorRange clamps the integral term's contribution to the output (default ±1.0)
    m_pid.setIntegratorRange(-1.0, 1.0);

    try {
      com.revrobotics.spark.config.SparkMaxConfig cfg = new com.revrobotics.spark.config.SparkMaxConfig();
      cfg.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
      cfg.smartCurrentLimit(60);
      cfg.openLoopRampRate(0.1);
      cfg.voltageCompensation((float) kNominalVoltage); // 11 V nominal so setVoltage() is consistent
      // Integrated encoder: explicit RPM for velocity (default is already RPM; 1.0 keeps native units)
      cfg.encoder.velocityConversionFactor(1.0);
      m_shooter.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    } catch (RuntimeException e) {
      Logger.recordOutput("Shooter/Errors", "[ShooterSubsystem] SparkMax configure failed: " + e.toString());
    }
  }

  /** Open-loop voltage (V). Clamped to ±11 V (ShooterConstants.kShooterVoltage). */
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -ShooterConstants.kShooterVoltage, ShooterConstants.kShooterVoltage);
    m_shooter.setVoltage(volts);
  }

  /** Convenience: -1..1 converted to voltage. All real output is voltage (setVoltage, setShootVoltage, setFeedVoltage). */
  public void setSpeed(double speed) {
    setVoltage(speed * ShooterConstants.kShooterVoltage);
  }

  /** Set velocity setpoint (RPM). Closed-loop: PID + FF hold target, output clamped to ±11 V. */
  public void setVelocitySetpointRpm(double targetRpm) {
    m_targetRpm = Math.abs(targetRpm);
  }

  /** Set RPM from distance (e.g. vision). Linear model, clamped to min/max. */
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

  /** Set feed voltage (V) for intake. When volts > 0, feeder runs at that voltage. Call with 0 to stop. */
  public void setFeedVoltage(double volts) {
    m_feedVoltage = volts > 0.0 ? Math.min(ShooterConstants.kShooterVoltage, volts) : 0.0;
  }

  /** Run shooter at fixed voltage (V) in shooting direction. Use for shoot command (e.g. 11 V). Call with 0 to use velocity setpoint instead. */
  public void setShootVoltage(double volts) {
    m_shootVoltage = MathUtil.clamp(volts, 0.0, ShooterConstants.kShooterVoltage);
  }

  /** Stop shooter and clear setpoints. */
  public void stop() {
    m_targetRpm = 0.0;
    m_feedVoltage = 0.0;
    m_shootVoltage = 0.0;
    m_shooter.stopMotor();
  }

  /** Velocity from NEO integrated encoder in motor RPM. Used for PID, FF, and telemetry. */
  public double getVelocity() {
    return m_shooter.getEncoder().getVelocity(); // getEncoder() = internal encoder; getVelocity() = RPM
  }

  /** Velocity in control convention: positive = shooting direction (equals -getVelocity() for our wiring). */
  public double getVelocityRpmForControl() {
    return -getVelocity();
  }

  /** For telemetry; same as getVelocity() with single motor. */
  public double getAverageVelocity() {
    return getVelocity();
  }

  public double getOutputCurrent() {
    return m_shooter.getOutputCurrent();
  }

  @Override
  public void periodic() {
    NetworkTable tuning = NetworkTableInstance.getDefault().getTable("Tuning");
    double p = tuning.getEntry("ShooterP").getDouble(ShooterConstants.kShooterP);
    double i = tuning.getEntry("ShooterI").getDouble(ShooterConstants.kShooterI);
    double d = tuning.getEntry("ShooterD").getDouble(ShooterConstants.kShooterD);
    if (p != m_pid.getP() || i != m_pid.getI() || d != m_pid.getD()) {
      m_pid.setPID(p, i, d);
      m_pid.setIntegratorRange(-1.0, 1.0);
    }

    // Feed (for intake) takes precedence; uses m_feedVoltage (set by IntakeCommand or default).
    if (m_feedVoltage > 0.0) {
      double volts = m_feedVoltage;
      m_shooter.setVoltage(volts);
      m_lastOutputPercent = volts / kNominalVoltage;
    } else if (m_shootVoltage > 0.0) {
      // Fixed voltage shoot: same direction as closed-loop (negative voltage = shooting direction).
      double volts = -m_shootVoltage;
      m_shooter.setVoltage(volts);
      m_lastOutputPercent = volts / kNominalVoltage;
    } else if (m_targetRpm > 1.0) {
      // Closed-loop velocity: reset PID when setpoint changes.
      if (m_targetRpm != m_lastTargetRpm) {
        m_pid.reset();
        m_lastTargetRpm = m_targetRpm;
      }
      // Integrated encoder RPM; invert so positive = shooting direction for PID setpoint comparison.
      double currentRpm = getVelocityRpmForControl();
      // PID output is dimensionless (P has units 1/RPM); treat as fraction and multiply by nominal voltage.
      double pidFraction = m_pid.calculate(currentRpm, m_targetRpm);
      double omegaRadPerSec = m_targetRpm * 2.0 * Math.PI / 60.0;
      double ffVolts = m_ff.calculate(omegaRadPerSec);
      double outVolts = -(pidFraction * kNominalVoltage + ffVolts);
      outVolts = MathUtil.clamp(outVolts, -kNominalVoltage, kNominalVoltage);
      m_shooter.setVoltage(outVolts);
      m_lastOutputPercent = outVolts / kNominalVoltage; // for telemetry / getCommandedVoltage()
    } else {
      m_lastTargetRpm = -1.0;
      m_shootVoltage = 0.0;
      m_shooter.setVoltage(0.0);
      m_lastOutputPercent = 0.0;
    }

    // Log integrated encoder and control state (velocity only; no position).
    Logger.recordOutput("Shooter/Velocity", getVelocity()); // raw encoder RPM
    Logger.recordOutput("Shooter/VelocityRpmForControl", getVelocityRpmForControl()); // value used in PID
    Logger.recordOutput("Shooter/TargetRpm", m_targetRpm);
    Logger.recordOutput("Shooter/OutputPercent", m_lastOutputPercent);
    Logger.recordOutput("Shooter/Current", getOutputCurrent());
  }

  public double getTargetRpm() {
    return m_targetRpm;
  }

  /** True when closed-loop target is set and current RPM >= 95% of target (quick recovery after first ball). */
  public boolean isAtSpeed() {
    if (m_targetRpm < 1.0) return false;
    double currentRpm = getVelocityRpmForControl();
    return currentRpm >= m_targetRpm * ShooterConstants.kShooterAtSpeedFraction;
  }

  public double getLastOutputPercent() {
    return m_lastOutputPercent;
  }

  /** Last commanded voltage (V). All output is voltage. */
  public double getCommandedVoltage() {
    return m_lastOutputPercent * kNominalVoltage;
  }
}
