package frc.robot.commands.Rst_Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.OdometrySubsystem;

/**
 * Restablece instantaneamente la odometria al origen del campo (0,0,0).
 */
public class ResetOdometryCommand extends InstantCommand {

  public ResetOdometryCommand(OdometrySubsystem odometry) {

    super(() -> {
      Pose2d origin = new Pose2d();
      odometry.resetOdometry(origin);
      edu.wpi.first.wpilibj.DataLogManager.log("[Command] Odometry reset to origin.");
    }, odometry);

  }
}