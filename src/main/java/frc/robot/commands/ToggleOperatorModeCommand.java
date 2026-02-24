package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.OperatorSubsystem;

/** Comando instantaneo que alterna el modo del OperatorSubsystem. */
public class ToggleOperatorModeCommand extends InstantCommand {
  public ToggleOperatorModeCommand(OperatorSubsystem operator) {
    super(() -> {
      operator.toggleMode();
      DataLogManager.log("ToggleOperatorMode invoked: " + operator.isOperatorMode());
    }, operator);
  }
}
