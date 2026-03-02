package frc.robot.commands.Intk_Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

/** Alterna el sentido del intake en el IntakeSubsystem. */
public class ToggleIntakeDirectionCommand extends InstantCommand {
  public ToggleIntakeDirectionCommand(IntakeSubsystem intake) {
    super(() -> {
      intake.toggleReverse();
    }, intake);
  }
}
