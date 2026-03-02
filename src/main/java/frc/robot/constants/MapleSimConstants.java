// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Poses and other settings for maple-sim (Team 5516 Iron Maple) simulation.
 * This is the single place to configure where the robot and field start in sim.
 *
 * <ul>
 *   <li><b>Robot initial pose</b> – Used when simulation starts so odometry and the sim chassis
 *       start at the same position.</li>
 *   <li><b>Hub poses</b> – Blue/Red hub positions (2026 Rebuilt) logged for AdvantageScope 3D field.</li>
 *   <li><b>FUEL on field</b> – Game piece positions come from maple-sim; we log them in MapleSimHandler.</li>
 * </ul>
 */
public final class MapleSimConstants {
  private MapleSimConstants() {}

  // ----- Robot initial pose (sim start) -----
  /** Initial robot X position in simulation (meters). */
  public static final double kSimInitialPoseX = 0.0;
  /** Initial robot Y position in simulation (meters). */
  public static final double kSimInitialPoseY = 0.0;
  /** Initial robot heading in simulation (degrees, 0 = +X). */
  public static final double kSimInitialPoseHeadingDeg = 0.0;

  /** Initial pose for the robot when simulation starts. Used to reset odometry so maple-sim and odometry match. */
  public static final Pose2d kSimInitialPose =
      new Pose2d(kSimInitialPoseX, kSimInitialPoseY, Rotation2d.fromDegrees(kSimInitialPoseHeadingDeg));

  // ----- Hub poses (2026 Rebuilt). Match org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub for consistency. -----
  /** Blue alliance hub center (RebuiltHub.blueHubPose). */
  public static final Pose3d kBlueHubPose =
      new Pose3d(4.5974, 4.034536, 1.5748, new Rotation3d());
  /** Red alliance hub center (RebuiltHub.redHubPose). */
  public static final Pose3d kRedHubPose =
      new Pose3d(11.938, 4.034536, 1.5748, new Rotation3d());
}
