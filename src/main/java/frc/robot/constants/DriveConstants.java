// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Drivetrain motor ports, geometry, feedforward gains, and simulation parameters.
 * Used by DriveSubsystem and PathPlanner. Set motor type (kBrushed/kBrushless) in DriveSubsystem to match hardware.
 */
public final class DriveConstants {
  private DriveConstants() {}

  // ----- CAN ports (right {1,2}, left {3,4}) -----
  public static final int kRightFrontMotorPort = 4;
  public static final int kRightRearMotorPort = 3;
  public static final int kLeftFrontMotorPort = 2;
  public static final int kLeftRearMotorPort = 1;
  public static final int[] kLeftMotorPorts = {kLeftFrontMotorPort, kLeftRearMotorPort};
  public static final int[] kRightMotorPorts = {kRightFrontMotorPort, kRightRearMotorPort};

  // ----- Teleop scaling -----
  /** Forward/back speed scale (1.0 = full). */
  public static final double kDriveSpeedScale = 0.8;
  /** Turn speed scale (1.0 = full). */
  public static final double kTurnSpeedScale = 0.7;

  // ----- Geometry (KitBot 2026: 6" wheels, ToughBox Mini 10.71:1, ~22 in track) -----
  public static final double kWheelDiameterMeters = 0.1524;
  public static final double kDriveGearRatio = 10.71;
  public static final double kTrackwidthMeters = 0.5588;

  // ----- Feedforward (from SysId: V, V/(m/s), V/(m/s²)) -----
  public static final double kDriveKS = 0.2;
  public static final double kDriveKV = 1.2;
  public static final double kDriveKA = 0.05;
  public static final double kDriveEstMaxSpeed = 3.0;

  // ----- Simulation -----
  public static final double kSimMassKg = 25.0;
  public static final double kSimMomentOfInertiaKgM2 = 2.1;

  // ----- Precision drive (driver left trigger) -----
  /** Scale applied when precision mode active (e.g. 0.5 = 50% speed). */
  public static final double kPrecisionDriveScale = 0.5;
}
