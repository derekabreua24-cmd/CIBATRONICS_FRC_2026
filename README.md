# FRC Robot Project

FRC 2026 Java robot code (WPILib, command-based). Differential drive, PathPlanner, AdvantageKit, vision (AprilTag), maple-sim (2026 Rebuilt FUEL).

## Build & run

- **Build:** `./gradlew build` (or use WPILib VS Code / Cursor).
- **Deploy:** Deploy to robot from IDE or `./gradlew deploy`.
- **Simulation:** Run in desktop sim; simulator (maple-sim) code is in `frc.robot.simulation`.

## Structure

- `frc.robot` – Robot, RobotContainer, Main.
- `frc.robot.commands` – Drive, intake, shooter, reset, etc.
- `frc.robot.subsystems` – Drive, Odometry, NavX, Vision, Intake, Shooter, Telemetry.
- `frc.robot.simulation` – Maple-sim handler and SimLaunchNoteCommand (sim only).
- `frc.robot.constants` – Drive, intake, shooter, vision, auto, operator constants.
