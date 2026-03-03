package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Stops intake and shooter immediately (single tap). Useful in match for emergencies or unjamming.
 */
public class StopMechanismsCommand extends InstantCommand {
  public StopMechanismsCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    super(() -> {
      intake.stop();
      shooter.stop();
    }, intake, shooter);
  }
}
