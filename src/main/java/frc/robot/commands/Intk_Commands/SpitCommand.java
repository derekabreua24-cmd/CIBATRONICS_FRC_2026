package frc.robot.commands.Intk_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * While held: runs intake in the spit direction (pushes fuel out). Uses shooter as feeder in reverse
 * direction if available; otherwise intake only. Release to stop.
 */
public class SpitCommand extends Command {
  private static final double kSpitVoltage = 6.0;

  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;

  public SpitCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    m_intake = intake;
    m_shooter = shooter;
    addRequirements(intake, shooter);
  }

  @Override
  public void initialize() {
    m_shooter.setFeedVoltage(0.0);
  }

  @Override
  public void execute() {
    // Positive voltage = spit out (opposite of IntakeCommand's -voltage for in)
    m_intake.runAtVoltage(kSpitVoltage);
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
