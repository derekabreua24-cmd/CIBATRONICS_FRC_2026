// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Fusión de pose por visión: desviaciones típicas para addVisionMeasurement (m, m, rad). */
public final class VisionConstants {
  private VisionConstants() {}

  public static final double kVisionStdDevXYMultiTagClose = 0.06;
  public static final double kVisionStdDevXYSingleOrFar = 0.2;
  public static final double kVisionStdDevThetaMultiTag = 0.05;
  public static final double kVisionStdDevThetaSingle = 0.12;
  public static final double kVisionFarDistanceMeters = 2.0;
  public static final double kVisionFarStdDevExtra = 0.08;
  public static final double kVisionVeryFarStdDevExtra = 0.15;
}
