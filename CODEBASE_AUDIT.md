# Codebase Audit

**Date:** 2026-02-28  
**Scope:** FRC robot project (WPILib 2026.2.1, REVLib 2026, maple-sim 0.4.0-beta, AdvantageKit, PathPlanner).

---

## 1. Build & configuration

| Item | Status | Notes |
|------|--------|--------|
| **build.gradle** | OK | WPILib 2026.2.1, Java 17, REVLib 2026.0.3, maple-sim + dyn4j; `sourceSets` excludes `com/**` to avoid vendor conflicts. |
| **settings.gradle** | OK | Plugin management and FRC home resolution. |
| **vendordeps/maple-sim.json** | OK | Matches `build.gradle` (maplesim-java 0.4.0-beta, dyn4j 5.0.2). |
| **Compilation** | OK | `./gradlew compileJava` succeeds. |

**Warnings (non-blocking):** 8× `[this-escape]` from calling `addRequirements(...)` in command/subsystem constructors. Common in WPILib; safe unless subclasses are misused. Optional fix: move `addRequirements` to a static factory or `configureBindings` where applicable.

---

## 2. Entry points & robot lifecycle

| Item | Status | Notes |
|------|--------|--------|
| **Main.java** | OK | Starts `Robot`; no static state. |
| **Robot.java** | OK | Extends `LoggedRobot`; scheduler in `robotPeriodic`; mode logging in `disabledInit`/`autonomousInit`/`teleopInit`/`testInit`; vision shutdown in `disabledInit`. |
| **simulationInit()** | Note | Calls `resetSimulationInitialPose()` then `autonomousInit()` so sim starts in auto (document if intentional for smoke tests). |
| **RobotContainer** | OK | Builds subsystems, vision, PathPlanner, bindings; sim reset callback registered; vision null-safe. |

---

## 3. Subsystems

### DriveSubsystem
- **Odometry:** `DifferentialDrivePoseEstimator`; heading from NavX (real) or maple-sim (sim) via `getSimHeading()`.
- **Sim:** No local integration; state comes from `MapleSimHandler` → `setSimStateFromMapleSim(pose)`; `getDesiredChassisSpeedsForSim()` used for chassis velocity command; reset callback syncs chassis pose.
- **PathPlanner:** `driveWithSpeeds(ChassisSpeeds, DriveFeedforwards)`; NT tuning fallbacks with one-time error logging.
- **SysId:** Mechanism and logging wired; LB + A/B/X/Y bindings.

### OdometrySubsystem
- **Periodic:** Uses sim heading when present, else NavX; `updateOdometryWithTime` with FPGA time.
- **Logging:** Logs every cycle (`Odometry/Update`). Consider reducing to debug or rate-limited to avoid log spam.

### IntakeSubsystem
- **API:** `run(speed)`, `toggleReverse()`, `stop()`, `getSetpoint()`; maple-sim uses `getSetpoint() > threshold` for intake sim.
- **Deprecation:** `setInverted` suppressed; consider migrating to REV config when convenient.

### ShooterSubsystem
- **Control:** Velocity setpoint (RPM); PID + feedforward in `periodic()`; tuning from NetworkTable `Tuning` (ShooterP/I/D).
- **Tuning:** TelemetrySubsystem writes Shuffleboard Shooter Tuning values into `Tuning`; ShooterSubsystem reads from `Tuning` — consistent.
- **Config:** SparkMax config (coast, current limit, ramp, voltage comp) in constructor with try/catch.

### NavXSubsystem
- **Null-safe:** All accessors handle `m_ahrs == null` (e.g. sim or missing hardware).

### VisionSubsystem
- **Layout:** Requires `AprilTagFieldLayout` + `Transform3d`; optional constructor throws.
- **Fusion:** `processDetections` → fused pose + std devs; `getLastPose` / `getLastTimestamp` / `getLastVisionStdDevs` for estimator.
- **Sim:** `setSimulationPoseAndDistance` used by MapleSimHandler; vision fusion in TelemetrySubsystem is disabled during teleop.

### TelemetrySubsystem
- **Vision fusion:** Only when `!isTeleop()` and vision enabled; reads last pose/timestamp/stdDevs and calls `addVisionMeasurement`. In sim, injected pose matches odometry (same source) — no conflict.
- **Tuning sync:** Writes Shooter (and other) tuning from Shuffleboard to `Tuning` table every cycle; ShooterSubsystem reads from same table.
- **Command logging:** `onCommandInitialize` / `onCommandFinish` / `onCommandInterrupt`; event log and Logger used safely.

---

## 4. Commands

| Command | Requirements | Null / edge cases |
|---------|--------------|--------------------|
| DriveCommand | drive | Default command; fine. |
| TurnToAngleCommand | drive | Reads TurnP/I/D/Tol from `Tuning` (TelemetrySubsystem populates). Driver X = turn to 90°. |
| IntakeCommand, UnjamCommand | intake | Require IntakeSubsystem. |
| ShooterCommand | shooter | Vision optional; uses RPM entry or distance-based RPM. |
| ResetGyroCommand | navx | — |
| ResetOdometryToVisionCommand | drive, vision | Only bound when `m_visionSubsystem != null`. |
| StopMechanismsCommand | intake, shooter | Stops both (operator B). |
| ToggleIntakeDirectionCommand | intake | Operator A; toggles intake direction (forward/reverse). |
| SimLaunchNoteCommand | none | No-op when `!isSimulation()`; uses odometry pose, chassis speeds, shooter RPM. |

---

## 5. Simulation (maple-sim)

| Item | Status | Notes |
|------|--------|--------|
| **MapleSimHandler** | OK | Uses `SimulatedArena.getInstance()` (2026 Rebuilt); `resetFieldForAuto()`; adds `KinematicChassisSim` and `IntakeSimulation` ("Fuel", OTB); only sets chassis speeds (no pose overwrite); reads pose after `arena.simulationPeriodic()` and calls `setSimStateFromMapleSim`; logs Fuel, hubs, intake/shooter; vision injection; `resetChassisPose` for odometry reset. |
| **KinematicChassisSim** | OK | Extends `AbstractDriveTrainSimulation`; empty `simulationSubTick`; pose from physics. |
| **SimLaunchNoteCommand** | OK | Builds `RebuiltFuelOnFly` per official API; trajectory callbacks; `addGamePieceProjectile` (arena calls `launch()`). |
| **Constants** | OK | `MapleSimConstants`: initial pose; hub poses match RebuiltHub (4.5974, 4.034536, 1.5748) and (11.938, 4.034536, 1.5748). |

---

## 6. Constants & config

- **DriveConstants:** Ports, geometry, feedforward, sim mass; used by drive and PathPlanner.
- **IntakeConstants, ShooterConstants, VisionConstants, AutoConstants, OperatorConstants:** Used consistently; no obvious conflicts.

---

## 7. Controller bindings

- **Driver:** LB+SysId (A/B/X/Y), Start=gyro reset, Back=vision reset (if vision present), RT=shooter, LT=intake, RB=stop, POV=unjam/reverse/toggle, A=SimLaunchNote (sim), X=turn 90°, B=drive-to-pose.
- **Operator:** B=stop, LT=intake, A=toggle direction, LB=unjam, RB=reverse intake, RT=shooter.

**Binding note:** Driver **A** is used for both:
- `leftBumper().and(a()).whileTrue(SysId forward)` and  
- `a().onTrue(SimLaunchNoteCommand)`.

So: A alone → launch projectile (sim). A+LB → SysId and one launch. To avoid launching when doing SysId, consider:  
`m_driverController.a().and(m_driverController.leftBumper().negate()).onTrue(new SimLaunchNoteCommand(...))`.

---

## 8. PathPlanner & autonomous

- AutoBuilder configured with odometry, reset, chassis speeds, `driveWithSpeeds`, PPLTV, alliance, drive.
- Logging callbacks set for current/target pose and active path.
- `getAutonomousCommand()`: Chooser; NavX reset; optional odometry reset from auto JSON (path start pose); reset-odom logic parses path waypoints; errors logged and fallback to `resetOdometry(new Pose2d())`.

---

## 9. Vision pipeline

- **RobotContainer:** Loads AprilTag layout from deploy (2026-rebuilt preferred); builds VisionSubsystem and UsbAprilTagProcessor; catches native/class errors and sets `m_visionSubsystem = null` or `m_usbProcessor = null`.
- **UsbAprilTagProcessor:** Not fully audited here; assumed to call `VisionSubsystem.processDetections` with correct timestamp.
- **TelemetrySubsystem:** Applies vision measurements only when not teleop and vision fusion enabled; uses last std devs when present.

---

## 10. Potential improvements (optional)

1. **OdometrySubsystem:** Reduce or rate-limit `Logger.recordOutput("Odometry/Update", ...)` to avoid log volume.
2. **Driver A in sim:** Restrict SimLaunchNote to when left bumper is not pressed so A+LB only runs SysId.
3. **this-escape warnings:** Optionally refactor commands/subsystems so `addRequirements` is not called in constructors that pass `this` (e.g. static factories or two-phase init).
4. **simulationInit auto start:** If auto-on-sim-start is not desired, remove or gate the `autonomousInit()` call in `Robot.simulationInit()`.

---

## 11. Summary

- **Build:** Compiles; vendordeps and dependencies consistent.
- **Structure:** Clear separation (subsystems, commands, constants, simulation); maple-sim used for arena, chassis, intake, and projectiles; odometry and drive sim state aligned with physics.
- **Null safety:** Vision, NavX, and optional tuning entries handled.
- **Logging:** AdvantageKit/Logger and TelemetrySubsystem used; one suggestion to reduce odometry log frequency.
- **Bindings:** Documented; one optional refinement for Driver A in sim.

No critical issues found; optional items above are small cleanups or behavior tweaks.
