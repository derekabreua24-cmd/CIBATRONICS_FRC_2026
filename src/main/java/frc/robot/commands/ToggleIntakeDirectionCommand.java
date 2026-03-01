package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

/** Toggle the intake direction state in the IntakeSubsystem. */
public class ToggleIntakeDirectionCommand extends InstantCommand {
  public ToggleIntakeDirectionCommand(IntakeSubsystem intake) {
    super(() -> {
      intake.toggleReverse();
    }, intake);
  }
}
