package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FeederConstants;

/** Feeder subsystem: moves note from intake toward shooter. Single SparkMax motor. */
public class FeederSubsystem extends SubsystemBase {
  private final SparkMax m_feeder =
      new SparkMax(FeederConstants.kFeederMotorPort, MotorType.kBrushed);

  public FeederSubsystem() {
    com.revrobotics.spark.config.SparkMaxConfig cfg =
        new com.revrobotics.spark.config.SparkMaxConfig();
    cfg.inverted(false);
    m_feeder.configure(
        cfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  /** Runs the feeder motor at the given output magnitude. */
  public void run(double speed) {
    m_feeder.set(speed);
  }

  /** Stops the feeder motor. */
  public void stop() {
    m_feeder.stopMotor();
  }

  @Override
  public void periodic() {}

  public double getSetpoint() {
    return m_feeder.get();
  }

  public double getOutputCurrent() {
    return m_feeder.getOutputCurrent();
  }
}
