// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Intake motor port, default speed, and unjam pulse duration.
 * Used by IntakeSubsystem and UnjamCommand.
 */
public final class IntakeConstants {
  private IntakeConstants() {}

  /** CAN ID for the intake motor (match motor type in IntakeSubsystem: kBrushed/kBrushless). */
  public static final int kIntakeMotorPort = 5;

  /** Default intake output magnitude (0..1). Used by IntakeCommand, UnjamCommand. */
  public static final double kDefaultSpeed = 0.8;

  /** Unjam (reverse pulse) configuration. */
  public static final class Unjam {
    private Unjam() {}

    /** Duration in seconds of reverse pulse for unjam (UnjamCommand). */
    public static final double kReverseDurationSec = 0.6;
  }
}
