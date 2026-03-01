// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Constantes del tren de rodaje (motores, geometría, feedforward, simulación). */
public final class DriveConstants {
  private DriveConstants() {}

  // IDs CAN. Motores derechos {1,2}, izquierdos {3,4}. kBrushless/kBrushed según hardware.
  public static final int kRightFrontMotorPort = 1;
  public static final int kRightRearMotorPort = 2;
  public static final int kLeftFrontMotorPort = 3;
  public static final int kLeftRearMotorPort = 4;
  public static final int[] kLeftMotorPorts = {kLeftFrontMotorPort, kLeftRearMotorPort};
  public static final int[] kRightMotorPorts = {kRightFrontMotorPort, kRightRearMotorPort};

  /** Escala de conducción (1.0 = velocidad máxima). */
  public static final double kDriveSpeedScale = 0.8;
  public static final double kTurnSpeedScale = 0.7;

  // KitBot 2026 (AM14U6): ruedas 6", ToughBox Mini 10.71:1, vía ~22 in.
  public static final double kWheelDiameterMeters = 0.1524;
  public static final double kDriveGearRatio = 10.71;
  public static final double kTrackwidthMeters = 0.5588;

  /** Feedforward desde SysId (V, V/(m/s), V/(m/s²)). */
  public static final double kDriveKS = 0.2;
  public static final double kDriveKV = 1.2;
  public static final double kDriveKA = 0.05;
  public static final double kDriveEstMaxSpeed = 3.0;

  /** Simulación: masa (kg) y momento de inercia (kg·m²). */
  public static final double kSimMassKg = 25.0;
  public static final double kSimMomentOfInertiaKgM2 = 2.1;

  /** Escala en modo precisión (driver gatillo izquierdo). 0.5 = 50 %. */
  public static final double kPrecisionDriveScale = 0.5;
}
