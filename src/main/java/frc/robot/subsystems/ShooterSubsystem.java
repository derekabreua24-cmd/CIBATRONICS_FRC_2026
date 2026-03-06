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
 * Closed-loop velocity control: PID + feedforward from target RPM, output clamped to ±12 V.
 * Uses voltage compensation (12 V nominal). Feed mode = 12 V reverse.
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
  /** When > 0, motor runs at 12 V reverse to feed fuel during intake. */
  private double m_feedVoltage = 0.0;
  private double m_lastOutputPercent = 0.0;

  public ShooterSubsystem() {
    m_shooter = new SparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless); // NEO: brushless, internal encoder
    m_pid.setIntegratorRange(-1.0 / ShooterConstants.kShooterP, 1.0 / ShooterConstants.kShooterP);

    try {
      com.revrobotics.spark.config.SparkMaxConfig cfg = new com.revrobotics.spark.config.SparkMaxConfig();
      cfg.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
      cfg.smartCurrentLimit(60);
      cfg.openLoopRampRate(0.1);
      cfg.voltageCompensation((float) kNominalVoltage); // 12 V nominal so setVoltage() is consistent with battery
      m_shooter.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    } catch (RuntimeException e) {
      Logger.recordOutput("Shooter/Errors", "[ShooterSubsystem] SparkMax configure failed: " + e.toString());
    }
  }

  /** Open-loop voltage (V). Clamped to ±12 V (ShooterConstants.kShooterVoltage). */
  public void setVoltage(double volts) {
    volts = Math.max(-ShooterConstants.kShooterVoltage, Math.min(ShooterConstants.kShooterVoltage, volts));
    m_shooter.setVoltage(volts);
  }

  /** Convenience: open-loop as fraction of 12 V (-1..1). */
  public void setSpeed(double speed) {
    setVoltage(speed * ShooterConstants.kShooterVoltage);
  }

  /** Set velocity setpoint (RPM). Closed-loop: PID + FF hold target, output clamped to ±12 V. */
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

  /** Set feed on/off for intake; motor runs in reverse at fixed 12 V. Any value > 0 uses 12 V. Call with 0 to stop. */
  public void setFeedVoltage(double volts) {
    m_feedVoltage = volts > 0.0 ? ShooterConstants.kShooterVoltage : 0.0;
  }

  /** Stop shooter and clear setpoints. */
  public void stop() {
    m_targetRpm = 0.0;
    m_feedVoltage = 0.0;
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

    // Feed (reverse for intake) takes precedence; always 12 V when active.
    if (m_feedVoltage > 0.0) {
      double volts = -kNominalVoltage;
      m_shooter.setVoltage(volts);
      m_lastOutputPercent = -volts / kNominalVoltage;
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
      outVolts = Math.max(-kNominalVoltage, Math.min(kNominalVoltage, outVolts)); // clamp to ±12 V
      m_shooter.setVoltage(outVolts);
      m_lastOutputPercent = outVolts / kNominalVoltage;
    } else {
      m_lastTargetRpm = -1.0;
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
}
