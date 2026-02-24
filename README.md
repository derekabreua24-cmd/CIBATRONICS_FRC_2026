# Final code 2 — Robot Project

This project is an FRC Java (WPILib) robot project targeting WPILib 2026. It includes:

- Command-based robot structure (Subsystems, Commands)
- PathPlanner integration (PathplannerLib 2026.1.2)
- Telemetry subsystem with Field2d and logging

Quick notes:

PathPlanner
-----------
- Use the PathPlanner application (matching version 2026.1.2) to design autos.
- From the PathPlanner app, use "Deploy to robot" to copy generated JSONs into the robot's deploy directory, or copy the JSONs manually into `src/main/deploy` before building/deploying the robot code.
- The robot code uses `AutoBuilder.buildAutoChooser()` to populate a Shuffleboard chooser called "PathPlanner Autos" on the Autonomous tab. Select an auto from that chooser before enabling autonomous.

Drive feedforward / characterization
------------------------------------
- This project contains placeholder feedforward constants in `Constants.DriveConstants`:
  - `kDriveKS`, `kDriveKV`, `kDriveKA` and `kDriveEstMaxSpeed`.
- These are placeholders. For accurate trajectory following, run WPILib robot characterization on your robot and replace those values with measured ks/kv/ka.
- The `DriveSubsystem.driveWithSpeeds(...)` method uses a small `DrivePhysics` utility that applies a `SimpleMotorFeedforward` calculation. Replace constants and tune the PathPlanner controller gains for best results.

Tests
-----
- A small unit test (`DrivePhysicsTest`) validates the physics helper. Run tests with Gradle:

```pwsh
./gradlew test
```

Next steps
----------
- Replace feedforward placeholders with characterization results.
- Tune the `PPLTVController` gain used by PathPlanner (default used in code is conservative).
- Optionally add more unit tests and CI to build and test on push.

