package frc.robot.commands.Intk_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.FeederConstants;

/** Runs intake and feeder in reverse while the button is held (e.g. for unjamming). */
public class ReverseIntakeCommand extends Command {
  private final IntakeSubsystem m_intake;
  private final FeederSubsystem m_feeder;

  public ReverseIntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder) {
    m_intake = intake;
    m_feeder = feeder;
    addRequirements(intake, feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.run(-IntakeConstants.kDefaultSpeed);
    m_feeder.run(-FeederConstants.kDefaultSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
