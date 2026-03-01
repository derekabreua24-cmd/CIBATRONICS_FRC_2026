package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.constants.IntakeConstants;

/** Ejecuta el intake en reversa mientras se mantiene el botón. */
public class ReverseIntakeCommand extends Command {
  private final IntakeSubsystem m_intake;

  public ReverseIntakeCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.run(-IntakeConstants.kDefaultSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
