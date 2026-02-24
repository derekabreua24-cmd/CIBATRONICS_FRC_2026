package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;

/** Ejecuta el motor del indexer a una velocidad fija mientras el comando este activo. */
public class IndexerCommand extends Command {
  private final IndexerSubsystem m_indexer;

  public IndexerCommand(IndexerSubsystem indexer) {
    m_indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_indexer.run(DriveConstants.kIndexerSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
