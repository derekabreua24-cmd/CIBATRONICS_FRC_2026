# Code Overview for Judges

This document gives a quick tour of the robot code so you can find subsystems, commands, and key design decisions.

---

## Project structure

```
frc.robot/
├── Main.java              Entry point; starts the robot (no logic here).
├── Robot.java              Lifecycle (init/periodic/disabled/auto/teleop); runs CommandScheduler.
├── RobotContainer.java     Wiring: subsystems, bindings, autonomous chooser, vision/pathplanner setup.
│
├── constants/             Tuning and hardware IDs (single place to change ports/speeds).
│   ├── OperatorConstants   Driver and operator controller ports.
│   ├── DriveConstants     Drivetrain motors, geometry, feedforward, sim.
│   ├── IntakeConstants     Intake motor port, speed, unjam duration.
│   ├── ShooterConstants   Shooter motors, PID/FF, distance-based RPM.
│   ├── VisionConstants    Vision pose fusion standard deviations.
│   └── AutoConstants      Gains for drive-to-pose and similar autos.
│
├── subsystems/            Hardware and state; expose methods used by commands.
│   ├── DriveSubsystem     Differential drive, odometry, pose estimator, SysId, sim.
│   ├── OdometrySubsystem  Single place that updates pose (gyro + encoders); resets for PathPlanner.
│   ├── NavXSubsystem      Gyro (Studica AHRS); null-safe if not present.
│   ├── IntakeSubsystem    Intake motor; reversible; toggle direction.
│   ├── ShooterSubsystem   Shooter motors, velocity PID + feedforward, distance-based RPM.
│   ├── VisionSubsystem    AprilTag pose fusion, std devs, distance to tag (for shooter).
│   ├── UsbAprilTagProcessor  Camera + AprilTag detection (separate thread); feeds VisionSubsystem.
│   └── TelemetrySubsystem Shuffleboard, Logger (AdvantageKit), vision fusion into pose estimator.
│
├── commands/              User actions and autos; all bindings in RobotContainer.configureBindings().
│   ├── Drv_Commands/      DriveCommand (default), DriveToPoseCommand, TurnToAngleCommand.
│   ├── Rst_Commands/      ResetGyroCommand, ResetOdometryToVisionCommand.
│   ├── IntakeCommand, ReverseIntakeCommand, UnjamCommand, ToggleIntakeDirectionCommand
│   ├── ShooterCommand, ShootSequenceCommand, StopMechanismsCommand
│   └── LogEventCommand
│
├── Camera_Calibration/     CameraCalibrationLoader: loads camera intrinsics and pose from deploy properties.
└── util/                   DrivePhysics (feedforward math), GeometryUtils (field/pose helpers).
```

---

## Design highlights

- **Command-based:** One default command (drive); everything else is button/trigger-bound. No reflection; WPILib APIs only (e.g. `SendableRegistry.getName()`, `AprilTagFieldLayout(Path)`).
- **Single odometry update:** Only `OdometrySubsystem.periodic()` updates the pose estimator; telemetry only adds vision measurements.
- **Vision:** AprilTag layout loaded from JSON in deploy (no enum/reflection). Optional; if camera fails, vision is disabled and sim can inject synthetic pose for testing.
- **PathPlanner:** Autos and paths live under `deploy/pathplanner/` (lowercase). Auto chooser on Shuffleboard; optional reset to path start when auto begins.
- **Constants:** All ports, speeds, and gains live in `frc.robot.constants`; subsystems and commands reference these for clarity and easy tuning.

---

## Where to look

| If you want to see…              | Open… |
|----------------------------------|--------|
| Button/trigger to command mapping | `RobotContainer.configureBindings()` |
| How pose is updated               | `OdometrySubsystem.periodic()`, `DriveSubsystem` (pose estimator) |
| How vision feeds pose             | `TelemetrySubsystem.periodic()` (vision fusion), `VisionSubsystem` |
| Drive and PathPlanner             | `DriveSubsystem.driveWithSpeeds()`, `RobotContainer` (AutoBuilder) |
| Shooter / distance-based RPM      | `ShooterSubsystem`, `ShooterCommand` / `ShootSequenceCommand` (use vision distance) |
| Tuning and constants              | `frc.robot.constants.*` |

---

## Building and running

- **Build:** `./gradlew build` (or use WPILib VS Code).
- **Simulate:** Run simulation, connect AdvantageScope to view poses and (if enabled) vision.
- **Deploy:** Standard WPILib deploy; ensure `deploy/` (camera config, pathplanner, AprilTag JSON) is included.
