package frc.robot.commands.Rst_Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.NavXSubsystem;

public class ResetGyroCommand extends InstantCommand {

  public ResetGyroCommand(NavXSubsystem navx) {
    super(() -> {
      navx.reset();
      DataLogManager.log("[Gyro] NavX yaw reset to zero.");
    }, navx);
  }
}