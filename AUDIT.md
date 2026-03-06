# FRC 2026 Project Audit

**Date:** 2025-03-05  
**Scope:** Full codebase review — structure, dependencies, subsystems, commands, constants, simulation, vision, and potential issues.

---

## 1. Project structure

| Area | Location | Notes |
|------|----------|------|
| Entry / lifecycle | `Robot.java`, `Main.java`, `RobotContainer.java` | LoggedRobot (AdvantageKit), command-based, single `RobotContainer` |
| Subsystems | `frc.robot.subsystems` | Drive, Odometry, NavX, Vision, Intake, Shooter, Telemetry, UsbAprilTagProcessor |
| Commands | `frc.robot.commands` (Drv_, Intk_, Sht_, Rst_, Log_) | Drive, Intake, Shooter, Reset, Log |
| Constants | `frc.robot.constants` | Drive, Intake, Shooter, Vision, Auto, Operator, MapleSim |
| Simulation | `frc.robot.simulation` | MapleSimHandler, KinematicChassisSim, SimLaunchNoteCommand |
| Utilities | `frc.robot.util` | DrivePhysics, GeometryUtils |
| Camera config | `frc.robot.Camera_Calibration` | CameraCalibrationLoader |

**Build / config**
- **build.gradle:** WPILib 2026.2.1, Java 17, REVLib 2026.0.3, maple-sim 0.4.0-beta, dyn4j 5.0.2. Source set excludes `com/**` to avoid vendor clashes. UTF-8 and `-Xlint:all` enabled.
- **vendordeps:** `maple-sim.json` (0.4.0-beta) present; REV expected via WPILib.
- **deploy:** AprilTag layout (`2026-rebuilt-andymark.json`, `apriltagfield_2026.json`), `camera/camera_calib.properties`, PathPlanner paths and autos present.

---

## 2. Subsystems summary

| Subsystem | Purpose | Null / edge handling |
|-----------|---------|------------------------|
| **DriveSubsystem** | 4× SparkMax diff drive, pose estimator, arcadeDrive, PathPlanner/SysId | Sim heading optional; brake mode, voltage scaling |
| **OdometrySubsystem** | Central pose API, gyro + encoders → estimator, reset, vision API (delegates to drive) | — |
| **NavXSubsystem** | AHRS (MXP SPI); rotation, yaw, pitch, roll | All getters guard `m_ahrs == null` |
| **VisionSubsystem** | AprilTag layout, processDetections, last pose/stddevs, distance | Handles null/empty detections |
| **UsbAprilTagProcessor** | USB camera + AprilTag detector thread, pushes to VisionSubsystem | Daemon thread, stop() joins and closes |
| **IntakeSubsystem** | Single SparkMax, runVoltage (fixed 12 V), reverse toggle | — |
| **ShooterSubsystem** | Single SparkMax, velocity PID+FF, feed at 12 V | — |
| **TelemetrySubsystem** | Shuffleboard/Field2d, logging, **vision fusion** (calls `m_drive.addVisionMeasurement`) | Guards `m_vision != null` |

---

## 3. Commands and bindings

| Command | Bound / used | Notes |
|---------|----------------|------|
| DriveCommand | Default for DriveSubsystem | Arcade drive, precision mode |
| TurnToAngleCommand | Driver X (90°) | Uses NavX, Tuning table for PID |
| IntakeCommand | Operator LT (whileTrue) | Intake + shooter feed |
| ToggleIntakeDirectionCommand | Operator A | Intake reverse |
| UnjamCommand | Operator LB | Intake reverse pulse |
| ShooterCommand | Operator RT (whileTrue) | RPM from vision/distance or tuning |
| StopMechanismsCommand | Operator B | Intake + shooter stop |
| ResetGyroCommand | Driver Start | NavX zero |
| ResetOdometryToVisionCommand | Driver Back (if vision != null) | Vision pose → drive.resetOdometry |
| SimLaunchNoteCommand | Driver A (no LB) | Sim only; no-op on robot |
| **ResetOdometryCommand** | **Not bound** | Resets to origin (0,0,0); **dead code** |
| **LogEventCommand** | **Not used** | Robot uses `m_telemetrySubsystem.logEvent()` instead; **dead code** |

SysId: Driver LB + A/B/X/Y (quasistatic/dynamic forward/reverse).

---

## 4. Constants and config

- **DriveConstants:** Ports 1–4, kNominalVoltage 12 V, kDriveSpeedScale -0.8, kTurnSpeedScale 0.7, geometry, feedforward, sim mass/inertia, precision scale.
- **IntakeConstants:** Port 5, kIntakeVoltage 12 V, unjam duration.
- **ShooterConstants:** Port 6, kShooterVoltage 12 V, feed 12 V, RPM curve, PID/FF, distance RPM.
- **VisionConstants:** Vision std devs and far/very-far thresholds.
- **AutoConstants:** Turn PID and tolerance (TurnToAngleCommand).
- **OperatorConstants:** Driver port 0, operator port 1.
- **MapleSimConstants:** Initial sim pose, Blue/Red hub poses.

No duplicate or conflicting constant definitions found. Single source of truth per concern.

---

## 5. Simulation (MapleSim)

- **MapleSimHandler:** Runs only when `RobotBase.isSimulation()`. Gets default 2026 Rebuilt arena, resets field, adds KinematicChassisSim and OverTheBumper IntakeSimulation ("Fuel"), then each tick: sets chassis speeds from `drive.getDesiredChassisSpeedsForSim()`, starts/stops intake by `|intake.getSetpoint()| > 0.02`, runs `arena.simulationPeriodic()`, syncs drive/odometry via `drive.setSimStateFromMapleSim(...)`, logs FUEL/hubs/intake/shooter, injects vision pose.
- **KinematicChassisSim:** Extends AbstractDriveTrainSimulation; velocity set by handler, no motor forces in simulationSubTick.
- **RobotContainer:** Registers `drive.setSimResetCallback(pose -> mapleSimHandler.resetChassisPose(pose))` so odometry reset in sim teleports the chassis body.
- **Robot.simulationInit():** Resets odometry to MapleSimConstants initial pose, then calls `autonomousInit()` (auto runs in sim for smoke test).
- **Dependency:** maplesim-java 0.4.0-beta and dyn4j in build.gradle and vendordeps; setup is consistent and ready.

---

## 6. Vision and odometry

- **Vision:** AprilTag layout loaded from deploy (several path candidates). VisionSubsystem holds layout and cameraToRobot; UsbAprilTagProcessor runs vision loop in daemon thread and calls `m_vision.processDetections(...)`. If layout missing or camera/processor fail (e.g. sim), vision is disabled (m_visionSubsystem / m_usbProcessor null).
- **Fusion:** TelemetrySubsystem.periodic() calls `m_drive.addVisionMeasurement(...)` when vision enabled and toggle is on (not OdometrySubsystem). OdometrySubsystem.addVisionMeasurement is never used; only DriveSubsystem’s estimator is used for fusion.
- **Reset to vision:** ResetOdometryToVisionCommand calls `drive.resetOdometry(pose, pose.getRotation())`, which triggers the sim reset callback when in sim.

---

## 7. Potential issues and improvements

1. **Dead code**
   - **ResetOdometryCommand** (`commands/Rst_Commands/ResetOdometryCommand.java`): Never scheduled or referenced. Either bind it (e.g. to a button) or remove it.
   - **LogEventCommand** (`commands/Log_Commands/LogEventCommand.java`): Replaced by `RobotContainer.logEvent()` → TelemetrySubsystem.logEvent(). Safe to delete or keep only if you want a command-based log trigger.

2. **Vision fusion API**
   - Vision measurements are applied in **TelemetrySubsystem** via `m_drive.addVisionMeasurement`. OdometrySubsystem also exposes `addVisionMeasurement` but nothing calls it. Consider either:
     - Using only OdometrySubsystem for vision (add measurement there and have it delegate to drive), or
     - Removing the unused method from OdometrySubsystem to avoid confusion.

3. **Driver A in teleop (real robot)**
   - Driver A (without LB) is bound to SimLaunchNoteCommand. On a real robot this command does nothing (initialize returns early). No functional bug, but the binding is sim-only in effect; consider documenting or moving to a sim-only binding if the framework allows.

4. **simulationInit() runs autonomous**
   - `Robot.simulationInit()` calls `autonomousInit()`, so the selected auto starts immediately in desktop sim. Intentional for smoke tests; just be aware when testing teleop-only in sim (e.g. cancel auto or don’t select one).

5. **PathPlanner / auto chooser**
   - getAutonomousCommand() parses the selected auto file from deploy to optionally reset odometry to the path’s first waypoint when "resetOdom" is true. Depends on PathPlanner JSON structure and file names; robust but somewhat brittle if PathPlanner format changes.

6. **TelemetrySubsystem constructor**
   - Accepts VisionSubsystem that may be null (when vision fails to init). All uses of m_vision in TelemetrySubsystem are null-checked. No change required.

7. **NavX**
   - NavXSubsystem handles AHRS init failure (m_ahrs null) in all public methods. No change required.

8. **Intake “forward” in sim**
   - MapleSimHandler now treats intake as active when `|intake.getSetpoint()| > 0.02`, so negative voltage (intake direction) correctly extends the intake and collects FUEL. Already fixed in a prior change.

---

## 8. Checklist summary

| Item | Status |
|------|--------|
| Build and dependencies | OK |
| Robot lifecycle and mode switches | OK |
| Subsystem null safety (NavX, Vision, Telemetry) | OK |
| Command bindings (except dead code) | OK |
| Constants centralised and consistent | OK |
| MapleSim init, periodic, reset callback | OK |
| Vision init, processor, fusion path | OK |
| Odometry reset (vision, auto, sim) and sim callback | OK |
| Unused commands (ResetOdometryCommand, LogEventCommand) | Consider remove or bind |
| Vision fusion ownership (Telemetry vs Odometry) | Consider clarifying or simplifying API |

---

**Conclusion:** The project is in good shape: clear structure, consistent use of constants and voltage-based intake/shooter, MapleSim and vision correctly wired, and null safety where needed. The main follow-ups are removing or binding the two unused commands and optionally clarifying the vision/odometry fusion API.
