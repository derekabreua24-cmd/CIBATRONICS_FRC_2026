package frc.robot.commands.Intk_Commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Runs inner intake (CAN 5) and shooter as feeder (outer intake) while active.
 * Shooter/feeder runs slower than inner intake. Releasing stops both.
 */
public class IntakeCommand extends Command {
  /** Inner intake (CAN 5) voltage (V). */
  private static final double kIntakeVoltage = 8.0;
  /** Shooter motor as feeder (V) during intake — lower than inner for gentler feed. */
  private static final double kShooterFeedVoltageDuringIntake = 4.0;

  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;

  public IntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    m_intake = intake;
    m_shooter = shooter;
    addRequirements(intake, shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.runAtVoltage(-kIntakeVoltage);
    m_shooter.setFeedVoltage(kShooterFeedVoltageDuringIntake);
    Logger.recordOutput("Intake/Events", "[IntakeCommand] Inner " + kIntakeVoltage + " V, feeder " + kShooterFeedVoltageDuringIntake + " V");
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_shooter.setFeedVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
