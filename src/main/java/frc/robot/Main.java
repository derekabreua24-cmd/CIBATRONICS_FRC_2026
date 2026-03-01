// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Application entry point. Do not add static state or initialization here.
 * Only change the class passed to startRobot if you use a different robot class.
 */
public final class Main {
  private Main() {}

  /**
   * Starts the robot. All initialization happens in Robot.robotInit() and RobotContainer.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
