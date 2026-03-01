// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Controller port IDs for driver and operator.
 * Used by RobotContainer when constructing CommandXboxController instances.
 */
public final class OperatorConstants {
  private OperatorConstants() {}

  /** USB port for driver controller (drive, gyro reset, vision reset, drive-to-pose). */
  public static final int kDriverControllerPort = 0;

  /** USB port for operator controller (intake, shooter, unjam, stop mechanisms). */
  public static final int kOperatorControllerPort = 1;
}
