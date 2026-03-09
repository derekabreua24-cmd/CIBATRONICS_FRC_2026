package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.ShooterConstants;

/**
 * Single-motor shooter (CAN ID 6): brushless NEO with internal encoder.
 * Closed-loop velocity control: PID + feedforward from target RPM, output clamped to ±11 V.
 * Uses voltage compensation (11 V nominal). Feed mode = 8 V (feeder/outer intake during intake).
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
  /** When > 0, motor runs at kShooterFeedVoltage (8 V) to feed during intake. */
  private double m_feedVoltage = 0.0;
  /** When > 0, motor runs at this voltage (V) in shooting direction (open-loop). Used by ShooterCommand for fixed 11 V shoot. */
  private double m_shootVoltage = 0.0;
  private double m_lastOutputPercent = 0.0;

  public ShooterSubsystem() {
    m_shooter = new SparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless); // NEO: brushless, internal encoder
    m_pid.setIntegratorRange(-1.0 / ShooterConstants.kShooterP, 1.0 / ShooterConstants.kShooterP);

    try {
      com.revrobotics.spark.config.SparkMaxConfig cfg = new com.revrobotics.spark.config.SparkMaxConfig();
      cfg.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
      cfg.smartCurrentLimit(60);
      cfg.openLoopRampRate(0.1);
      cfg.voltageCompensation((float) kNominalVoltage); // 11 V nominal so setVoltage() is consistent
      m_shooter.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    } catch (RuntimeException e) {
      Logger.recordOutput("Shooter/Errors", "[ShooterSubsystem] SparkMax configure failed: " + e.toString());
    }
  }

  /** Open-loop voltage (V). Clamped to ±11 V (ShooterConstants.kShooterVoltage). */
  public void setVoltage(double volts) {
    volts = Math.max(-ShooterConstants.kShooterVoltage, Math.min(ShooterConstants.kShooterVoltage, volts));
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

  /** Set feed on/off for intake; motor runs at kShooterFeedVoltage (8 V). Any value > 0 enables. Call with 0 to stop. */
  public void setFeedVoltage(double volts) {
    m_feedVoltage = volts > 0.0 ? ShooterConstants.kShooterFeedVoltage : 0.0;
  }

  /** Run shooter at fixed voltage (V) in shooting direction. Use for shoot command (e.g. 11 V). Call with 0 to use velocity setpoint instead. */
  public void setShootVoltage(double volts) {
    m_shootVoltage = Math.max(0.0, Math.min(ShooterConstants.kShooterVoltage, volts));
  }

  /** Stop shooter and clear setpoints. */
  public void stop() {
    m_targetRpm = 0.0;
    m_feedVoltage = 0.0;
    m_shootVoltage = 0.0;
    m_shooter.stopMotor();
  }

  /** Velocity from NEO internal encoder (RPM). Only velocity is used for control and logging. */
  public double getVelocity() {
    return m_shooter.getEncoder().getVelocity();
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
    }

    // Feed (for intake) takes precedence; uses kShooterFeedVoltage (8 V) when active.
    if (m_feedVoltage > 0.0) {
      double volts = ShooterConstants.kShooterFeedVoltage;
      m_shooter.setVoltage(volts);
      m_lastOutputPercent = volts / ShooterConstants.kShooterFeedVoltage;
    } else if (m_shootVoltage > 0.0) {
      // Fixed voltage shoot (e.g. 11 V from ShooterCommand).
      double volts = m_shootVoltage;
      m_shooter.setVoltage(volts);
      m_lastOutputPercent = volts / kNominalVoltage;
    } else if (m_targetRpm > 1.0) {
      // Closed-loop velocity: reset PID when setpoint changes.
      if (m_targetRpm != m_lastTargetRpm) {
        m_pid.reset();
        m_lastTargetRpm = m_targetRpm;
      }
      // NEO internal encoder: getVelocity() is motor RPM; invert so positive = shooting direction.
      double currentRpm = -getVelocity();
      double pidPercent = m_pid.calculate(currentRpm, m_targetRpm);
      double omegaRadPerSec = m_targetRpm * 2.0 * Math.PI / 60.0;
      double ffVolts = m_ff.calculate(omegaRadPerSec);
      double outVolts = -(pidPercent * kNominalVoltage + ffVolts);
      outVolts = Math.max(-kNominalVoltage, Math.min(kNominalVoltage, outVolts)); // clamp to ±11 V
      m_shooter.setVoltage(outVolts);
      m_lastOutputPercent = outVolts / kNominalVoltage;
    } else {
      m_lastTargetRpm = -1.0;
      m_shootVoltage = 0.0;
      m_shooter.setVoltage(0.0);
      m_lastOutputPercent = 0.0;
    }

    // Log to AdvantageKit/DataLogManager (and thus AdvantageScope) every cycle (velocity only).
    Logger.recordOutput("Shooter/Velocity", getVelocity());
    Logger.recordOutput("Shooter/TargetRpm", m_targetRpm);
    Logger.recordOutput("Shooter/OutputPercent", m_lastOutputPercent);
    Logger.recordOutput("Shooter/Current", getOutputCurrent());
  }

  public double getTargetRpm() {
    return m_targetRpm;
  }

  public double getLastOutputPercent() {
    return m_lastOutputPercent;
  }

  /** Last commanded voltage (V). All output is voltage. */
  public double getCommandedVoltage() {
    return m_lastOutputPercent * kNominalVoltage;
  }
}
