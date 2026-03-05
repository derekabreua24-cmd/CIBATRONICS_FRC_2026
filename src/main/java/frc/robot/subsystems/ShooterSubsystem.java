package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.ShooterConstants;

/** Single-motor shooter (CAN ID 6). Velocity control via PID + feedforward. */
public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_shooter;
  private final SimpleMotorFeedforward m_feedforward;
  private final PIDController m_pid;
  private double m_targetRpm = 0.0;
  /** When > 0, motor runs at -this voltage (reverse) to feed note during intake. */
  private double m_feedVoltage = 0.0;
  private double m_lastOutputPercent = 0.0;

  public ShooterSubsystem() {
    m_shooter = new SparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushed);

    m_feedforward = new SimpleMotorFeedforward(
        ShooterConstants.kShooterKS,
        ShooterConstants.kShooterKV,
        ShooterConstants.kShooterKA);

    m_pid = new PIDController(
        ShooterConstants.kShooterP,
        ShooterConstants.kShooterI,
        ShooterConstants.kShooterD);

    try {
      com.revrobotics.spark.config.SparkMaxConfig cfg = new com.revrobotics.spark.config.SparkMaxConfig();
      cfg.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
      cfg.smartCurrentLimit(60);
      cfg.openLoopRampRate(0.1);
      cfg.voltageCompensation((float) ShooterConstants.kShooterVoltage);
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

  /** Set velocity setpoint (RPM). PID + feedforward applied in periodic(). */
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
      double volts = -ShooterConstants.kShooterVoltage;
      m_shooter.setVoltage(volts);
      m_lastOutputPercent = -volts / ShooterConstants.kShooterVoltage;
    } else if (m_targetRpm > 1.0) {
      // Shooter motor reversed: invert encoder reading and output so PID still holds target RPM.
      double currentRpm = -getVelocity();
      double pidPercent = m_pid.calculate(currentRpm, m_targetRpm);
      double targetRadPerSec = m_targetRpm * 2.0 * Math.PI / 60.0;
      double ffVolts = m_feedforward.calculate(targetRadPerSec);
      double outVolts = -(pidPercent * ShooterConstants.kShooterVoltage + ffVolts);
      outVolts = Math.max(-ShooterConstants.kShooterVoltage, Math.min(ShooterConstants.kShooterVoltage, outVolts));
      m_shooter.setVoltage(outVolts);
      m_lastOutputPercent = outVolts / ShooterConstants.kShooterVoltage;
    } else {
      m_shooter.setVoltage(0.0);
      m_lastOutputPercent = 0.0;
    }
  }

  public double getTargetRpm() {
    return m_targetRpm;
  }

  public double getLastOutputPercent() {
    return m_lastOutputPercent;
  }
}
