// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Constantes del intake (motor, velocidad, unjam). */
public final class IntakeConstants {
  private IntakeConstants() {}

  /** ID CAN del motor del intake. kBrushed/kBrushless según IntakeSubsystem. */
  public static final int kIntakeMotorPort = 5;
  /** Velocidad por defecto (porcentaje de salida). */
  public static final double kIntakeSpeed = 0.8;
  /** Duración (s) del pulso de reversa para desatascar (UnjamCommand). */
  public static final double kUnjamReverseDurationSec = 0.6;
}
