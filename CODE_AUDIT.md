Cl# Code Audit Report — FRC Robot (2026)

Audit date: 2026-02-28. Scope: all Java sources, entry points, subsystems, commands, and utilities.

**Status:** All listed issues have been addressed in code (see "Fixed" notes below).

---

## Executive Summary

The codebase is a well-structured FRC command-based robot with PathPlanner, AdvantageKit, vision (AprilTag), and differential drive. Several **critical** and **medium** issues were found and have been fixed: double odometry updates, shooter RPM tuning not wired, and documentation/consistency issues. Recommendations are ordered by severity.

---

## Critical Issues

### 1. Odometry updated twice per cycle (bug) — **Fixed**

**Location:** `OdometrySubsystem.periodic()` and `TelemetrySubsystem.periodic()`.

- **OdometrySubsystem** (line 36): `m_drive.updateOdometry(heading);`
- **TelemetrySubsystem** (line 311): `m_drive.updateOdometry(m_navx.getRotation2d());`

`DifferentialDrivePoseEstimator.update()` is therefore called twice each cycle. That can double encoder deltas or otherwise corrupt the pose.

**Fixed:** Call removed from `TelemetrySubsystem.periodic()`; odometry updates only in `OdometrySubsystem.periodic()`.

---

### 2. Shooter RPM tuning entry is null when bindings are configured — **Fixed**

**Location:** `RobotContainer` constructor.

- `configureBindings()` is called at the start of the constructor (line 95).
- `m_shooterRpmEntry` is created later (line 209) when building the Tuning tab.
- `ShooterCommand(m_shooterSubsystem, m_shooterRpmEntry)` is registered inside `configureBindings()`, so it always receives `null`.
- `ShooterCommand` then uses the default RPM only; the "Shooter RPM" widget on the Tuning tab has no effect.

**Fixed:** Tuning tab and `m_shooterRpmEntry` are now created at the start of `RobotContainer()` before `configureBindings()`.

---

### 3. Shooter P/I/D and feed tuning not updated at runtime — **Fixed**

**Location:** `TelemetrySubsystem` constructor vs `ShooterSubsystem.periodic()` / `ShootSequenceCommand.execute()`.

- `TelemetrySubsystem` writes `ShooterP`, `ShooterI`, `ShooterD`, `FeedPulseSec`, `FeedPauseSec`, `FeedContinuous` to the `"Tuning"` NetworkTable **only in the constructor**.
- `ShooterSubsystem.periodic()` and `ShootSequenceCommand` read those keys from `"Tuning"` each cycle.
- Changing the Shuffleboard "Shooter Tuning" or feed values at runtime does not update the table, so runtime tuning has no effect.

**Fixed:** `TelemetrySubsystem` now stores Shooter and feed tuning entries as fields and, in `periodic()`, copies all tuning values (Turn, Shooter, Feed, Est) from Shuffleboard into the `"Tuning"` NetworkTable so subsystems see runtime changes.

---

## Medium Issues

### 4. Constants comment disagrees with CAN IDs — **Fixed**

**Location:** `Constants.java` (DriveConstants).

- Comment says: "left motors at CAN IDs {1,2}, right motors at {3,4}".
- Code defines: `kRightFrontMotorPort = 1`, `kRightRearMotorPort = 2`, `kLeftFrontMotorPort = 3`, `kLeftRearMotorPort = 4` (right = 1,2; left = 3,4).
- So the comment has left/right swapped.

**Fixed:** Comment updated to "Right motors at CAN IDs {1,2}, left motors at {3,4}." Added note in Constants and subsystem comments to use kBrushless for NEO/brushless and kBrushed for brushed.

---

### 5. Motor type: Brushed vs brushless — **Documented**

**Location:** `DriveSubsystem.java`, `IntakeSubsystem.java`.

- Drive and intake use `MotorType.kBrushed` for SparkMax.
- Many FRC drivetrains and intakes use NEO or other brushless motors (`MotorType.kBrushless`).

**Fixed:** Constants and DriveSubsystem/IntakeSubsystem now document that motor type must match hardware (kBrushless for NEO, kBrushed for brushed). Teams can switch the constant in code when hardware is confirmed.

---

### 6. DrivePhysics voltage clamp when `estMaxSpeed == 0` — **Fixed**

**Location:** `DrivePhysics.computeTankVoltages()`.

- Voltages are clamped to ±12 V only when `estMaxSpeed > 0`.
- If `estMaxSpeed` is 0 (or negative), outputs are unclamped and could exceed safe range.

**Fixed:** Voltages are now always clamped to ±12 V regardless of `estMaxSpeed`.

---

### 7. RobotContainer AprilTag export comment — **Fixed**

**Location:** `RobotContainer.java` (else branch after loading field layout from deploy).

- Comment says: "If we successfully loaded a built-in layout (e.g., 2026)..."
- That block runs when a **deploy** file was found and loaded, not when using the built-in layout.

**Fixed:** Comment updated to describe deploy-loaded layout export.

---

## Minor / Style Issues

### 8. Duplicate Javadoc opener in Robot.java — **Fixed**

**Location:** `Robot.java` lines 58–59.

- Two consecutive `/**` lines start the same doc comment.

**Fixed:** Extra `/**` removed.

---

### 9. Duplicate import in RobotContainer.java — **Fixed**

**Location:** `RobotContainer.java`.

- `GenericEntry` is imported twice (e.g. lines 35 and 42).

**Fixed:** Duplicate `GenericEntry` import removed.

---

### 10. NavXSubsystem indentation — **Fixed**

**Location:** `NavXSubsystem.java` — `getPitch()` and `getRoll()`.

- Methods are not indented consistently with the rest of the class.

**Fixed:** `getPitch()`, `getRoll()`, and `reset()` indentation corrected; constructor Logger calls indented.

---

### 11. Placeholder DrivePhysicsTest in main sources — **Fixed**

**Location:** `src/main/java/frc/robot/util/util/DrivePhysicsTest.java`.

- A placeholder or duplicate test lives under `main`; the real test is in `src/test/java/frc/robot/util/DrivePhysicsTest.java`.

**Fixed:** `src/main/java/frc/robot/util/util/DrivePhysicsTest.java` removed; tests remain in `src/test/java`.

---

### 12. IntakeSubsystem deprecation suppression — **Fixed**

**Location:** `IntakeSubsystem` constructor — `@SuppressWarnings("deprecation")`.

- It’s unclear what is deprecated (e.g. constructor, or a SparkMax API).

**Fixed:** Javadoc added: "REV SparkMax setInverted() is deprecated in favor of config-based inversion; suppress until migrated."

---

## Positive Notes

- Clear separation: subsystems, commands, util, and constants are organized.
- AdvantageKit logging and PathPlanner 2026 integration are used consistently.
- Vision (AprilTag) and odometry reset are handled with null checks and fallbacks.
- DrivePhysics is pure and unit-tested.
- Camera calibration and deploy paths are handled for RoboRIO.
- Graceful handling when vision or NavX fail to initialize.

---

## Logic verification (2026 WPILib / PathPlanner)

Verified against WPILib 2026.2.2 and PathPlanner 2026 API:

- **DifferentialDrivePoseEstimator**: `update(gyroAngle, leftMeters, rightMeters)` and `updateWithTime(time, gyroAngle, leftMeters, rightMeters)` take **cumulative** wheel distances in meters. The code uses `rotationsToMeters(getLeftAveragePosition())` (encoder position → meters); correct. `resetPosition(gyroAngle, leftPos, rightPos, pose)` is called with 0, 0 after zeroing encoders; correct.
- **addVisionMeasurement(pose, timestampSeconds)**: Timestamp must use the same epoch as the pose estimator’s time source. The code now uses `updateWithTime(Timer.getFPGATimestamp(), ...)` in `OdometrySubsystem.periodic()`, and the vision thread uses `Timer.getFPGATimestamp()` when submitting measurements; both use FPGA time, so vision fusion is correct.
- **PathPlanner AutoBuilder**: Expects pose supplier, reset (single `Pose2d`), robot-relative chassis speeds, and drive output. `OdometrySubsystem::getPose` and `resetOdometry(Pose2d)` match. `getChassisSpeeds()` returns robot-relative (vx, 0, omega); `driveWithSpeeds(ChassisSpeeds, DriveFeedforwards)` is used correctly.
- **PPLTVController**: Constructor parameter is **dt** (discretization timestep in seconds), not a gain. Fixed: default is 0.02 s (20 ms loop), widget renamed to "PPLTV dt (s)", and value is clamped to (0, 0.1].
- **ChassisSpeeds / kinematics**: Omega positive = CCW (turn left). `omega = (rightVel - leftVel) / trackWidth` and `leftVel = vx - omega*trackWidth/2`, `rightVel = vx + omega*trackWidth/2` match WPILib differential drive convention.
- **Vision pose**: `robotPoseField = cameraPoseField.transformBy(cameraToRobot.inverse())` correctly converts camera pose to robot center pose in field frame.
- **DriveSubsystem**: No longer depends on `NavXSubsystem`; odometry and heading are owned by `OdometrySubsystem`, which calls `updateOdometryWithTime()` with FPGA time and gyro heading.

---

## Suggested Fix Order (all completed)

1. ~~Remove duplicate `updateOdometry()` from `TelemetrySubsystem.periodic()`.~~
2. ~~Create Tuning tab and `m_shooterRpmEntry` before `configureBindings()`.~~
3. ~~Sync Shooter P/I/D and feed tuning from Shuffleboard to Tuning table in `TelemetrySubsystem.periodic()`.~~
4. ~~Fix Constants comment, Robot.java doc comment, RobotContainer comment and duplicate import.~~
5. ~~Document motor types; always clamp in DrivePhysics; clean up NavX indentation; remove placeholder test; document IntakeSubsystem deprecation.~~

---

## AprilTag & Vision Verification (2026)

**Status:** Verified and corrected where needed.

| Component | Status | Notes |
|-----------|--------|--------|
| **AprilTag field layout** | OK | **Modern:** 1) Built-in 2026 tried first via `AprilTagFieldLayout.loadField(f)` for any `AprilTagFields` whose name contains `"2026"` (WPILib 2026.2+: `k2026RebuiltAndymark`, `k2026RebuiltWelded`). 2) Deploy JSON via constructor `new AprilTagFieldLayout(Path)` (no reflection). 3) Fallback `kDefaultField`. Null layout → vision disabled. |
| **AprilTag processor** | Fixed | Detector now uses **tag36h11** (FRC 2026 REBUILT standard). Previously used tag16h5, which would not detect 2026 field tags. |
| **Camera calibration** | OK | `CameraCalibrationLoader` loads `deploy/camera/camera_calib.properties` (fx, fy, cx, cy, tagSizeMeters, optional cameraToRobot). **tagSizeMeters** set to **0.1651** (6.5 in) for 2026 REBUILT. |
| **Camera → robot transform** | OK | Properties define camera position/rotation in robot frame (robot-to-camera). Vision uses `cameraPoseField.transformBy(m_cameraToRobot.inverse())` to get robot pose, which is correct. |
| **VisionSubsystem** | OK | `processDetection(tagId, tagToCamera, timestamp)` → `getTagPose(tagId)` from layout, then tag→camera→robot chain; sanity filter \|x\|,\|y\| &lt; 20 m. |
| **UsbAprilTagProcessor** | OK | USB camera, CvSink, AprilTagDetector + AprilTagPoseEstimator; separate vision thread; config (threads, decimate, sigma). |
| **Vision → pose estimator** | OK | `TelemetrySubsystem.periodic()` calls `m_drive.addVisionMeasurement(pose, timestamp)` when vision enabled and not teleop; timestamps use FPGA time. |
| **Reset to vision** | OK | Back button runs `ResetOdometryToVisionCommand` (resets odometry to last vision pose + current NavX heading). |
| **Simulation** | OK | Vision init wrapped in try/catch for `UnsatisfiedLinkError`/`NoClassDefFoundError`; sim continues with vision disabled. |
| **Shutdown** | OK | `Robot.disabledInit()` calls `RobotContainer.shutdownVision()` → `UsbAprilTagProcessor.stop()` (stops thread, closes detector, releases Mats). |

**Latest AprilTag field layout (reference)**

- **Official JSON format:** Two top-level keys: `"tags"` (array of `{ "ID": n, "pose": { "translation": { x, y, z }, "rotation": { "quaternion": { W, X, Y, Z } } } }`) and `"field"` (`"length"` and `"width"` in meters). FRC coordinates: NWU, origin at bottom-right of blue alliance wall. Your `deploy/apriltagfield_2026.json` matches this and includes `"field": { "length": 16.541, "width": 8.069 }`.
- **WPILib 2026.2+ built-in fields:** `AprilTagFields.k2026RebuiltAndymark`, `AprilTagFields.k2026RebuiltWelded`. Prefer `AprilTagFieldLayout.loadField(AprilTagFields)` over the deprecated `AprilTagFields.loadAprilTagLayoutField()`. Custom layouts: use constructor `new AprilTagFieldLayout(Path)` or `AprilTagFieldLayout(String path)`.

**Implemented optional improvements**

1. **Vision measurement standard deviations** — `DriveSubsystem.addVisionMeasurement(pose, timestamp, stdDevs)` forwards to the pose estimator. `VisionSubsystem` computes per-measurement std devs from tag count and average distance (see `VisionConstants`); `TelemetrySubsystem` uses `getLastVisionStdDevs()` and calls the 3-arg overload when present.
2. **Multi-tag fusion** — `VisionSubsystem.processDetections(List<VisionDetection>, timestamp)` fuses all detections in a frame: average x, y, and rotation; std devs are smaller for 2+ tags and when tags are closer. `UsbAprilTagProcessor` collects all detections per frame and calls `processDetections` once with a frame timestamp.
3. **Frame capture timestamp** — Vision loop takes `Timer.getFPGATimestamp()` at the start of each iteration (before `grabFrame`), so the same timestamp is used for all detections in that frame and is closer to capture time.
4. **2026 built-in field** — `RobotContainer` first iterates `AprilTagFields.values()` for any name containing `"2026"` and uses `loadField(f)` if present; then tries deploy JSON via `new AprilTagFieldLayout(found)` (modern constructor; no reflection); then falls back to `kDefaultField`.
5. **Log vision pose to AdvantageKit** — `VisionSubsystem` logs `Vision/RobotPose` (Pose2d) in `processDetections`; add this key to AdvantageScope Poses to view the vision pose on the field.

---

## Final logic verification (one fix applied)

| Area | Verification | Result |
|------|--------------|--------|
| **Odometry update** | Only `OdometrySubsystem.periodic()` calls `updateOdometryWithTime()`; `TelemetrySubsystem` does not. | OK |
| **Encoder → meters** | `rotationsToMeters`: distance = rotations × π×D / gearRatio. `rpmToMetersPerSecond`: same relation for velocity. | OK |
| **ChassisSpeeds** | `vx = (leftVel + rightVel)/2`, `omega = (rightVel - leftVel)/trackWidth` (positive omega = CCW). | OK |
| **DrivePhysics** | `leftVel = vx − ω×L/2`, `rightVel = vx + ω×L/2`; feedforward and ±12 V clamp. | OK |
| **Vision pose** | `cameraPoseField = tagPoseField.transformBy(tagToCamera)`; `robotPoseField = cameraPoseField.transformBy(cameraToRobot.inverse())`. Properties store camera-in-robot-frame, so inverse gives robot from camera. | OK |
| **Vision fusion** | Applied only when `!isTeleop()` and vision enabled; timestamp and optional std devs passed through. | OK |
| **DriveCommand** | `rot` negated so stick right → robot turns right (WPILib arcadeDrive positive = left). | OK |
| **PathPlanner** | AutoBuilder: pose supplier, reset, chassis speeds, driveWithSpeeds; `resetOdometry(Pose2d)` used as callback. | OK |
| **Reset odometry** | **Fix:** `OdometrySubsystem.resetOdometry(pose)` now uses `pose.getRotation()` so PathPlanner’s reset pose (including heading) is applied. Previously it used `m_navx.getRotation2d()`, which ignored the path’s intended starting rotation. | Fixed |
| **Shooter** | PID(measurement, setpoint); FF from target rad/s; output clamped to ±1; `m_targetRpm > 1.0` gate. | OK |
| **Reset to vision** | `ResetOdometryToVisionCommand` uses vision pose position + NavX heading (intentional: vision for xy, gyro for theta). | OK |
