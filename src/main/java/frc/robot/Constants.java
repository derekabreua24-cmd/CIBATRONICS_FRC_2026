// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * La clase Constants centraliza constantes numéricas o booleanas del robot.
 * No usar para otra cosa. Todas las constantes deben ser públicas estáticas.
 * No incluir lógica funcional.
 *
 * <p>Se recomienda importar estáticamente esta clase (o una interna) donde se usen las constantes.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  // Puerto del controlador del operador (intake/shooter).
    public static final int kOperatorControllerPort = 1;
  }

  public static class DriveConstants {
  // IDs CAN de los controladores del tren de rodaje. Este robot usa 6 ruedas
  // accionadas por 4 motores (dos por lado). Actualizar si el cableado difiere.
  // Disposición actual: motores derechos CAN {1,2}, izquierdos {3,4}.
  // Tipo de motor: kBrushless para NEO/brushless; kBrushed para brushed. Ajustar en DriveSubsystem/IntakeSubsystem.
    public static final int kRightFrontMotorPort = 1;
    public static final int kRightRearMotorPort = 2;
    public static final int kLeftFrontMotorPort = 3;
    public static final int kLeftRearMotorPort = 4;
    public static final int[] kLeftMotorPorts = {kLeftFrontMotorPort, kLeftRearMotorPort};
    public static final int[] kRightMotorPorts = {kRightFrontMotorPort, kRightRearMotorPort};

    // ID CAN del motor del intake (tipo debe coincidir con IntakeSubsystem: kBrushed o kBrushless).
    public static final int kIntakeMotorPort = 5;

  // Puertos del shooter (frontal/trasero); actualizar si los IDs CAN difieren.
  public static final int kShooterFrontMotorPort = 7;
  public static final int kShooterRearMotorPort = 8;
  public static final double kShooterSpeed = 0.9;
  // RPM máxima aproximada en vacío del shooter (ajustar a la ficha del motor).
  public static final double kShooterMaxRPM = 5700.0;
  // Ganancias de feedforward del shooter (V, V/(rad/s), V/(rad/s²)); valores por defecto para flywheel Neo; afinar en robot.
  public static final double kShooterKS = 0.2;
  public static final double kShooterKV = 0.02;
  public static final double kShooterKA = 0.001;

    // Escala de conducción (1.0 = velocidad máxima).
    public static final double kDriveSpeedScale = 0.8;
    public static final double kTurnSpeedScale = 0.7;

  // ----- Especificaciones oficiales KitBot 2026 (AM14U6) -----
  // Ruedas HiGrip 6" (FIRST/AndyMark); ToughBox Mini S 10.71:1 opcional.
  // Vía: config cuadrada AM14U6 ~22 in (0.5588 m); medir si difiere.
  public static final double kWheelDiameterMeters = 0.1524;  // 6 pulgadas
  public static final double kDriveGearRatio = 10.71;
  public static final double kTrackwidthMeters = 0.5588;     // ~22 in; medir para tu chasis

  // Feedforward del tren de rodaje desde SysId (V, V/(m/s), V/(m/s²)). Sustituir por resultados de CHARACTERIZATION.md.
  public static final double kDriveKS = 0.2;
  public static final double kDriveKV = 1.2;
  public static final double kDriveKA = 0.05;

  // Velocidad lineal máxima (m/s) para limitar tensión. 10.71:1 CIM + ruedas 6" ≈ 3 m/s.
  public static final double kDriveEstMaxSpeed = 3.0;

  // Simulación: KitBot 2026 con batería/paragolpes ~50 lb → ~25 kg; J para vía ~0.56 m, chasis 25 kg.
  public static final double kSimMassKg = 25.0;
  public static final double kSimMomentOfInertiaKgM2 = 2.1;

    // Velocidad por defecto del intake (porcentaje de salida).
    public static final double kIntakeSpeed = 0.8;
  // Indexer eliminado de esta compilación; constantes relacionadas eliminadas.
  }

  /** RPM del shooter en función de la distancia: RPM = base + pendiente×distancia (m), limitado. Afinar para la curva del speaker. */
  public static class ShooterDistanceConstants {
    public static final double kShooterRpmAt0M = 3200.0;
    public static final double kShooterRpmPerMeter = 250.0;
    public static final double kShooterRpmMin = 2800.0;
    public static final double kShooterRpmMax = 5400.0;
  }

  /** Fusión de pose por visión: desv. tip. para addVisionMeasurement (m, m, rad). Mayor valor = confiar menos en visión. */
  public static class VisionConstants {
    /** Desv. tip. (m) en x/y con varios tags y cerca. */
    public static final double kVisionStdDevXYMultiTagClose = 0.06;
    /** Desv. tip. (m) en x/y con un solo tag o lejos. */
    public static final double kVisionStdDevXYSingleOrFar = 0.2;
    /** Desv. tip. (rad) en theta con varios tags. */
    public static final double kVisionStdDevThetaMultiTag = 0.05;
    /** Desv. tip. (rad) en theta con un solo tag. */
    public static final double kVisionStdDevThetaSingle = 0.12;
    /** Distancia (m) a partir de la cual se aumenta la desv. tip. (tag lejos). */
    public static final double kVisionFarDistanceMeters = 2.0;
    /** Desv. tip. extra en x/y cuando la distancia media al tag > kVisionFarDistanceMeters. */
    public static final double kVisionFarStdDevExtra = 0.08;
    /** Desv. tip. extra en x/y cuando la distancia media al tag > 3 m. */
    public static final double kVisionVeryFarStdDevExtra = 0.15;
  }
}
