// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Feeder motor port and default speed.
 * Used by FeederSubsystem and FeederCommand.
 */
public final class FeederConstants {
  private FeederConstants() {}

  /** CAN ID for the feeder motor (feeds note from intake to shooter). */
  public static final int kFeederMotorPort = 6;

  /** Default feeder output magnitude (0..1). */
  public static final double kDefaultSpeed = 0.8;
}
