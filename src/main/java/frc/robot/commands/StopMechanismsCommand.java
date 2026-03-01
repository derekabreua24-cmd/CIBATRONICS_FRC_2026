package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Detiene intake y shooter de inmediato (un tap). Útil en partido para emergencias o desbloquear.
 */
public class StopMechanismsCommand extends InstantCommand {
  public StopMechanismsCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    super(() -> {
      intake.stop();
      shooter.stop();
    }, intake, shooter);
  }
}
