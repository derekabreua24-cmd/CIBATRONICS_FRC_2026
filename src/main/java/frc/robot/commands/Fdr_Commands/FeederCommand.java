package frc.robot.commands.Fdr_Commands;

import frc.robot.subsystems.FeederSubsystem;
import frc.robot.constants.FeederConstants;
import edu.wpi.first.wpilibj2.command.Command;

/** Runs the feeder motor at default speed while the command is active. */
public class FeederCommand extends Command {
  private final FeederSubsystem m_feeder;

  public FeederCommand(FeederSubsystem feeder) {
    m_feeder = feeder;
    addRequirements(feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_feeder.run(FeederConstants.kDefaultSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
