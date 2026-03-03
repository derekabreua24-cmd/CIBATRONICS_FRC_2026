package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;

/**
 * Stops intake, shooter, and feeder immediately (single tap). Useful in match for emergencies or unjamming.
 */
public class StopMechanismsCommand extends InstantCommand {
  public StopMechanismsCommand(
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      FeederSubsystem feeder) {
    super(
        () -> {
          intake.stop();
          shooter.stop();
          feeder.stop();
        },
        intake,
        shooter,
        feeder);
  }
}
