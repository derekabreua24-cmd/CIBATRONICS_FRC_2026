# Subsystems, Commands, and Simulation Audit

Audit completed after full logic pass. All listed items were verified or fixed.

---

## Subsystems

### DriveSubsystem
- **Odometry**: `rotationsToMeters` and `rpmToMetersPerSecond` use wheel circumference and gear ratio correctly. `updateOdometry` / `updateOdometryWithTime` and `resetOdometry(heading, 0, 0, pose)` after zeroing encoders match WPILib 2026 API.
- **ChassisSpeeds**: `vx = (leftVel + rightVel)/2`, `omega = (rightVel - leftVel)/trackWidth` (robot-relative, CCW positive). `driveWithSpeeds` uses `DrivePhysics.computeTankVoltages` and optional PathPlanner feedforward; voltages clamped in DrivePhysics.
- **Display**: `periodic()` only reads and displays pose; no duplicate odometry update.
- **Simulation**: See Simulation section below.

### OdometrySubsystem
- Single source of odometry update: `updateOdometryWithTime(Timer.getFPGATimestamp(), heading)` so vision timestamps align. `resetOdometry(Pose2d)` uses current NavX heading and forwards to drive. `addVisionMeasurement` forwarded to drive. Logic correct.

### NavXSubsystem
- Null-safe: all accessors check `m_ahrs == null`. `getRotation2d()`, `zeroHeading()`, `reset()` correct. No logic issues.

### IntakeSubsystem
- `run(speed)`: applies `m_reversed ? -speed : speed`. `toggleReverse()` flips state. `stop()` stops motor. Logic correct.

### ShooterSubsystem
- **Fixes applied**: (1) When `m_targetRpm <= 1.0`, `periodic()` now sets motor to 0 and `m_lastOutputPercent = 0` so the subsystem doesn’t keep commanding after a logical “stop.” (2) `stop()` now sets `m_targetRpm = 0` before stopping the motor so subsequent `periodic()` calls don’t re-apply PID output.
- PID + feedforward: gains from NetworkTable "Tuning"; `calculate(currentRpm, m_targetRpm)`; feedforward from `targetRadPerSec`; output clamped to [-1, 1]. Logic correct.

### VisionSubsystem
- `computeRobotPose`: tag pose → camera pose via `tagToCamera`, then robot pose via `cameraToRobot.inverse()`. Sanity filter rejects |x| or |y| > 20 m. Thread-safe storage with `AtomicReference` and volatile timestamp. Logic correct.

### UsbAprilTagProcessor
- Vision loop uses `Timer.getFPGATimestamp()` for measurements (FPGA epoch for pose estimator). `stop()` sets `m_running = false`, joins thread, closes detector, releases Mats. No logic issues.

### OperatorSubsystem
- Toggle and Shuffleboard entry; no periodic sync from widget to `m_operatorMode` (toggle-only design). Acceptable.

### TelemetrySubsystem
- Syncs Shuffleboard tuning entries to NetworkTable "Tuning" in `periodic()` so ShooterSubsystem and others see runtime changes. Vision fusion only when not teleop and when enabled. Null checks for `m_vision` and `m_operatorController`. Logic correct.

---

## Commands

### DriveCommand
- Arcade: `-getLeftY()` for forward (Xbox convention), deadband, squared inputs, slew limiters, `arcadeDrive(fwd, rot)`. `end()` zeros. Requirements: drive. Correct.

### DriveToPoseCommand
- P control to target pose: distance and heading error, normalized angle error, forward capped by `m_maxSpeed`, rotation clamped. `isFinished()` on position and angle tolerance. `end()` zeros. Requirements: drive, odometry. Correct.

### TurnToAngleCommand
- PID with continuous input [-180, 180], tolerance, min output for friction. Reads gains from "Tuning". `end()` zeros and resets PID. Requirements: drive. Correct.

### ResetGyroCommand
- Calls `navx.reset()`. Instant, requires navx. Correct.

### ResetOdometryCommand
- Resets odometry to origin `new Pose2d()`. Instant, requires odometry. Correct.

### ResetOdometryToVisionCommand
- Uses last vision pose and current NavX heading; calls `drive.resetOdometry(pose, navx.getRotation2d())`. Requirements: drive, vision. Correct.

### IntakeCommand / ReverseIntakeCommand
- Intake: `run(kIntakeSpeed)` while active, `stop()` in `end()`. Reverse: `run(-kIntakeSpeed)`. IntakeSubsystem applies `m_reversed` inside `run()`, so reverse command always spins opposite to forward. Correct.

### ShooterCommand
- `initialize()` sets RPM from entry (or default); `execute()` refreshes from entry; `end()` calls `shooter.stop()`. With ShooterSubsystem fix, `stop()` clears setpoint so periodic doesn’t re-command. Correct.

### ShootSequenceCommand
- Starts shooter and intake in `initialize()`; execute keeps shooter setpoint and reads Feed* from "Tuning"; pulse/pause state is internal (indexer removed); `end()` stops both. Correct.

### ToggleIntakeDirectionCommand / ToggleOperatorModeCommand / LogEventCommand
- Instant commands; correct requirements and side effects.

---

## Simulation

### DriveSubsystem.simulationPeriodic()
- **API**: `setInputs(voltsL, voltsR)`; we pass `get() * 12.0` (percent → volts). `update(0.02)` for 20 ms loop. `getLeftPositionMeters()` / `getRightPositionMeters()` are wheel distances; conversion to motor rotations: `(meters / (π*D)) * gearRatio`. Matches WPILib 2026 DifferentialDrivetrainSim.
- **Constructor**: `DCMotor.getCIM(4)`, gearing, J (2.1), mass (7.5), wheel radius, track width, null measurement std devs. Order and units correct.
- **Gyro**: Not updated from sim in this class. Comment added: odometry heading comes from NavX; if the NavX supports simulation (e.g. setAngle), feed `m_driveSim.getHeading()` there for correct pose in sim.

---

## Summary of Code Changes

1. **ShooterSubsystem**
   - In `periodic()`, when `m_targetRpm <= 1.0`, set motor to 0 and `m_lastOutputPercent = 0`.
   - In `stop()`, set `m_targetRpm = 0` before stopping the motor.

2. **DriveSubsystem**
   - `simulationPeriodic()`: comments for setInputs (volts), wheel-to-motor conversion, and gyro sim note; minor formatting.

No other logic errors were found in subsystems, commands, or simulation.
