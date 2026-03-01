// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Standard deviations for vision pose fusion (addVisionMeasurement).
 * Units: X/Y in meters, theta in radians. Tighter values = more trust in vision.
 */
public final class VisionConstants {
  private VisionConstants() {}

  /** XY std dev when multiple tags are visible and close. */
  public static final double kVisionStdDevXYMultiTagClose = 0.06;
  /** XY std dev for single tag or far tags. */
  public static final double kVisionStdDevXYSingleOrFar = 0.2;
  /** Theta std dev when multiple tags are visible. */
  public static final double kVisionStdDevThetaMultiTag = 0.05;
  /** Theta std dev for single tag. */
  public static final double kVisionStdDevThetaSingle = 0.12;
  /** Distance (m) above which tags are considered "far" (extra uncertainty). */
  public static final double kVisionFarDistanceMeters = 2.0;
  /** Extra XY std dev for far tags. */
  public static final double kVisionFarStdDevExtra = 0.08;
  /** Extra XY std dev for very far tags (> 3 m). */
  public static final double kVisionVeryFarStdDevExtra = 0.15;
}
