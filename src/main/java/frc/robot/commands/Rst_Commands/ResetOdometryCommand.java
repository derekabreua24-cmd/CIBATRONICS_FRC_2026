package frc.robot.commands.Rst_Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.Logger;
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
  Logger.recordOutput("Commands/Reset", "[Command] Odometry reset to origin.");
    }, odometry);

  }
}