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

  // ----- Motor port (single shooter motor: brushless NEO, internal encoder) -----
  public static final int kShooterMotorPort = 6;

  // ----- Voltage and RPM -----
  /** Fixed voltage (V) for shooter (velocity control and output clamp). */
  public static final double kShooterVoltage = 11.0;
  /** Alias; same as kShooterVoltage. */
  public static final double kShooterNominalVoltage = kShooterVoltage;
  /** Default as fraction of max RPM for dashboard/tuning. 1.0 = absolute max. */
  public static final double kShooterSpeed = 0.7;
  public static final double kShooterMaxRPM = 5700.0;

  /** Feed voltage (V) when shooter runs as feeder (outer intake) during intake. */
  public static final double kShooterFeedVoltage = 9.0;

  /** Consider "at speed" when current RPM >= this fraction of target (e.g. 0.95 = 95%). */
  public static final double kShooterAtSpeedFraction = 0.95;
  /** RPM tolerance for "at speed" (secondary; primary is kShooterAtSpeedFraction). */
  public static final double kShooterAtSpeedToleranceRpm = 150.0;
  /** Delay (s) after shooter is at speed before running the feeder (0.5–0.8 s typical). */
  public static final double kShooterFeedDelayAfterAtSpeedSec = 0.65;

  // ----- Feedforward (SimpleMotorFeedforward: kS in V, kV in V/(rad/s), kA in V/(rad/s²)) -----
  public static final double kShooterKS = 0.2;
  public static final double kShooterKV = 0.02;
  public static final double kShooterKA = 0.001;

  // ----- Velocity PID (error in RPM; output is dimensionless fraction, multiplied by kShooterVoltage to get volts) -----
  /** P: 1/RPM so (RPM error)*P = fraction. Tuned for ~4000–5000 RPM. */
  public static final double kShooterP = 0.00035;
  /** I: removes steady-state RPM error. */
  public static final double kShooterI = 0.00002;
  /** D: usually 0 for velocity loop (noise). */
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
