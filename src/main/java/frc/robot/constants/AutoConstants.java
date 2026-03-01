// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Gains and tolerances for autonomous drive-to-pose, turn-to-angle, and similar commands.
 * Used by DriveToPoseCommand and TurnToAngleCommand.
 */
public final class AutoConstants {
  private AutoConstants() {}

  // ----- Drive to pose (proportional only) -----
  /** Proportional gain for linear (forward) error. */
  public static final double kDriveToPoseKpLinear = 1.2;
  /** Proportional gain for angular error. */
  public static final double kDriveToPoseKpAng = 2.0;

  // ----- Turn to angle (PID, degrees in, rotation out -0.8..0.8) -----
  /** Turn PID: proportional gain (tuned for ~0.8 output at ~25–30° error). */
  public static final double kTurnP = 0.035;
  /** Turn PID: integral gain (removes steady-state error). */
  public static final double kTurnI = 0.001;
  /** Turn PID: derivative gain (damping). */
  public static final double kTurnD = 0.002;
  /** Turn finished when error within this many degrees. */
  public static final double kTurnToleranceDeg = 2.0;
}
