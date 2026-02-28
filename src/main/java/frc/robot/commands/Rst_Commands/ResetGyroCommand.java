package frc.robot.commands.Rst_Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.NavXSubsystem;

public class ResetGyroCommand extends InstantCommand {

  public ResetGyroCommand(NavXSubsystem navx) {
    super(() -> {
      navx.reset();
      Logger.recordOutput("Commands/Reset", "[Gyro] NavX yaw reset to zero.");
    }, navx);
  }
}