// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Constantes del shooter (motores, velocidad, feedforward, RPM por distancia). */
public final class ShooterConstants {
  private ShooterConstants() {}

  public static final int kShooterFrontMotorPort = 7;
  public static final int kShooterRearMotorPort = 8;
  public static final double kShooterSpeed = 0.9;
  public static final double kShooterMaxRPM = 5700.0;
  /** Feedforward (V, V/(rad/s), V/(rad/s²)). */
  public static final double kShooterKS = 0.2;
  public static final double kShooterKV = 0.02;
  public static final double kShooterKA = 0.001;

  /** RPM en función de la distancia: base + pendiente×distancia (m), limitado. */
  public static class ShooterDistanceConstants {
    private ShooterDistanceConstants() {}
    public static final double kShooterRpmAt0M = 3200.0;
    public static final double kShooterRpmPerMeter = 250.0;
    public static final double kShooterRpmMin = 2800.0;
    public static final double kShooterRpmMax = 5400.0;
  }
}
