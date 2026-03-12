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

  /** Fixed voltage (V) for intake when using run()/runVoltage(). Was 6 V (too slow); 10–12 V for full speed. */
  public static final double kIntakeVoltage = 10.0;
  /** Alias for backwards compatibility; same as kIntakeVoltage. */
  public static final double kIntakeMaxVoltage = kIntakeVoltage;
  public static final double kDefaultVoltage = kIntakeVoltage;

  /** Open-loop PID for feeder: target voltage, no velocity feedback. P=1 so output = setpoint; I/D=0. */
  public static final double kIntakeFeederP = 1.0;
  public static final double kIntakeFeederI = 0.0;
  public static final double kIntakeFeederD = 0.0;
  /** Nominal voltage (V) for clamping PID output. */
  public static final double kIntakeNominalVoltage = 12.0;

  /** Unjam (reverse pulse) configuration. */
  public static final class Unjam {
    private Unjam() {}

    /** Duration in seconds of reverse pulse for unjam (UnjamCommand). */
    public static final double kReverseDurationSec = 0.6;
  }
}
