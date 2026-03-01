// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  // Port for the dedicated operator controller (intake/shooter)
    public static final int kOperatorControllerPort = 1;
  }

  public static class DriveConstants {
  // CAN IDs for the drivetrain motor controllers. This robot uses a
  // 6-wheel drivetrain mechanically (6 wheels) driven by 4 motors
  // (two motors per side). Update these arrays if your wiring differs.
  // Current layout: right motors at CAN IDs {1,2}, left motors at {3,4}.
  // Motor type: use MotorType.kBrushless for NEO/brushless; kBrushed for brushed. Update DriveSubsystem/IntakeSubsystem to match hardware.
    public static final int kRightFrontMotorPort = 1;
    public static final int kRightRearMotorPort = 2;
    public static final int kLeftFrontMotorPort = 3;
    public static final int kLeftRearMotorPort = 4;
    public static final int[] kLeftMotorPorts = {kLeftFrontMotorPort, kLeftRearMotorPort};
    public static final int[] kRightMotorPorts = {kRightFrontMotorPort, kRightRearMotorPort};

    // CAN ID for the intake motor (type must match IntakeSubsystem: kBrushed or kBrushless)
    public static final int kIntakeMotorPort = 5;

  // Shooter ports (front/rear) - update if your CAN IDs differ
  public static final int kShooterFrontMotorPort = 7;
  public static final int kShooterRearMotorPort = 8;
  public static final double kShooterSpeed = 0.9;
  // Approximate maximum free-speed RPM for the shooter motors (set to your motor spec)
  public static final double kShooterMaxRPM = 5700.0;
  // Feedforward gains for shooter (units: V, V/(rad/s), V/(rad/s^2))
  // These are approximate defaults for a Neo-driven flywheel; tune on robot.
  public static final double kShooterKS = 0.2;
  public static final double kShooterKV = 0.02;
  public static final double kShooterKA = 0.001;

    // Drive scaling (1.0 = full speed)
    public static final double kDriveSpeedScale = 0.8;
    public static final double kTurnSpeedScale = 0.7;

  // ----- 2026 KitBot (AM14U6) official specs -----
  // 6" HiGrip wheels (FIRST/AndyMark); ToughBox Mini S 10.71:1 optional ratio.
  // Track width: AM14U6 square config ~22 in (0.5588 m); measure yours if different.
  public static final double kWheelDiameterMeters = 0.1524;  // 6 inches
  public static final double kDriveGearRatio = 10.71;
  public static final double kTrackwidthMeters = 0.5588;     // ~22 in, measure for your frame

  // Drive feedforward from SysId (V, V/(m/s), V/(m/s²)). Replace with CHARACTERIZATION.md results.
  public static final double kDriveKS = 0.2;
  public static final double kDriveKV = 1.2;
  public static final double kDriveKA = 0.05;

  // Max linear speed (m/s) for voltage clamping. 10.71:1 CIM + 6" wheels ≈ 3 m/s.
  public static final double kDriveEstMaxSpeed = 3.0;

  // Simulation: 2026 KitBot with battery/bumpers ~50 lb → ~25 kg; J for ~0.56 m track, 25 kg chassis.
  public static final double kSimMassKg = 25.0;
  public static final double kSimMomentOfInertiaKgM2 = 2.1;

  // Simulation: field boundary (meters). 2026 REBUILT playable 651.2"×317.7" → 16.54 m × 8.07 m; origin center; margin for bumper.
  public static final double kSimFieldHalfLengthX = 8.2;   // 16.4 m total
  public static final double kSimFieldHalfWidthY = 4.0;    // 8 m total

  // Simulation: obstacles {minX, maxX, minY, maxY} in meters. Rough tower/outpost footprints (~0.6 m); robot stops on overlap.
  public static final double[][] kSimObstacles = {
      {-5.0, -4.4, -0.3, 0.3},   // left-side structure
      {4.4, 5.0, -0.3, 0.3}      // right-side structure
  };

  // Simulation: game piece physics — tuned for realistic intake/shooter behavior (note-like pickup, ~12+ m/s launch).
  public static final double kSimBallPickupRadiusMeters = 0.22;   // note center within ~9" of intake
  public static final double kSimIntakeOffsetXMeters = 0.28;      // intake center ahead of robot center
  public static final double kSimShooterLaunchSpeedMetersPerSec = 14.0;  // typical FRC speaker-range launch
  public static final double kSimShooterOffsetXMeters = 0.32;      // shooter exit ahead of robot center
  public static final double kSimShooterRpmThreshold = 4000.0;     // "at speed" when near target RPM
  public static final double kSimShotCooldownSec = 0.32;           // time between launched notes (feeder cycle)
  public static final double kSimGravityMetersPerSecSq = 9.81;

    // Default intake running speed (percent output)
    public static final double kIntakeSpeed = 0.8;
  // Indexer removed from this build - related constants deleted
  }

  /** Shooter distance-based RPM: RPM = base + slope×distance (m), clamped. Tune for your speaker curve. */
  public static class ShooterDistanceConstants {
    public static final double kShooterRpmAt0M = 3200.0;
    public static final double kShooterRpmPerMeter = 250.0;
    public static final double kShooterRpmMin = 2800.0;
    public static final double kShooterRpmMax = 5400.0;
  }
}
