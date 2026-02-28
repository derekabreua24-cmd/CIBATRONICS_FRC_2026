package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Indexer subsystem stub.
 *
 * The project was requested to remove the physical indexer; keep a lightweight stub to
 * avoid compile errors in places where the class is still referenced. This stub performs
 * no hardware actions and returns safe default values.
 */
public class IndexerSubsystem extends SubsystemBase {
  public IndexerSubsystem() {
    // Intentionally empty: real hardware removed.
  }

  /** No-op run. */
  public void run(double percent) {
    // no-op
  }

  /** No-op stop. */
  public void stop() {
    // no-op
  }

  @Override
  public void periodic() {
    // no periodic work
  }

  public double getSetpoint() {
    return 0.0;
  }

  public double getEncoderPosition() {
    return 0.0;
  }

  public double getEncoderVelocity() {
    return 0.0;
  }

  public double getOutputCurrent() {
    return 0.0;
  }
}
