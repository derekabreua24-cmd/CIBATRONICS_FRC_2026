// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Shooter motor ports, default speed, feedforward gains, and distance-based RPM curve.
 * Used by ShooterSubsystem and shooter commands.
 */
public final class ShooterConstants {
  private ShooterConstants() {}

  // ----- Motor port (single shooter motor) -----
  public static final int kShooterMotorPort = 6;

  // ----- Default speed and cap -----
  public static final double kShooterSpeed = 0.9;
  public static final double kShooterMaxRPM = 5700.0;

  /** Speed (0..1) for shooter when used as feeder during intake. Motor runs in reverse to pull note in; flip sign in code if your mechanism is opposite. */
  public static final double kShooterFeedSpeed = 0.5;

  // ----- Feedforward (V, V/(rad/s), V/(rad/s²)) -----
  public static final double kShooterKS = 0.2;
  public static final double kShooterKV = 0.02;
  public static final double kShooterKA = 0.001;

  // ----- Velocity PID (error in RPM, output added to FF percent) -----
  /** Shooter velocity PID: proportional (tuned for ~4000–5000 RPM range). */
  public static final double kShooterP = 0.00035;
  /** Shooter velocity PID: integral (removes steady-state RPM error). */
  public static final double kShooterI = 0.00002;
  /** Shooter velocity PID: derivative (usually 0 for velocity loop). */
  public static final double kShooterD = 0.0;

  /** Distance-based RPM: base + slope×distance (m), clamped to min/max. */
  public static final class ShooterDistanceConstants {
    private ShooterDistanceConstants() {}

    public static final double kShooterRpmAt0M = 3200.0;
    public static final double kShooterRpmPerMeter = 250.0;
    public static final double kShooterRpmMin = 2800.0;
    public static final double kShooterRpmMax = 5400.0;
  }
}
