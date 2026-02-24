package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;

/** Ejecuta el motor del intake a una velocidad fija mientras el comando este activo. */
public class IntakeCommand extends Command {
  private final IntakeSubsystem m_intake;

  public IntakeCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.run(DriveConstants.kIntakeSpeed);
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
