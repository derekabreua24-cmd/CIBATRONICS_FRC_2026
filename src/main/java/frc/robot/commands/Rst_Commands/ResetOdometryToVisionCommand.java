package frc.robot.commands.Rst_Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ResetOdometryToVisionCommand extends InstantCommand {

  public ResetOdometryToVisionCommand(
      DriveSubsystem drive,
      VisionSubsystem vision,
      NavXSubsystem navx) {

    super(() -> {

      vision.getLastPose().ifPresentOrElse(
            pose -> {
            drive.resetOdometry(pose, navx.getRotation2d());
            Logger.recordOutput("Commands/Reset", "[Vision] Odometry reset to vision pose: " + pose);
          },
          () -> Logger.recordOutput("Commands/Reset", "[Vision] No vision pose available - odometry not reset.")
      );

    }, drive, vision);

  }
}