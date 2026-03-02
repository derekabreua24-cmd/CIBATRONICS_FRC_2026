# Maple-Sim Integration (Team 5516 Iron Maple)

**[maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim)** from the Shenzhen Robotics Alliance (Team 5516 Iron Maple) adds a 2D physics world (dyn4j) to simulation: obstacles, game pieces, and arena physics.

## What’s in this project

- **Vendordep:** `vendordeps/maple-sim.json` – uses **v0.4.0-beta** (2026 Rebuilt; [releases](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/releases)).
- **Simulation:** In `RobotContainer.simulationPeriodic()`, when in simulation we call `SimulatedArena.getInstance().simulationPeriodic()` so the maple-sim world updates each cycle (only in sim, not on a real robot).
- **Drivetrain:** This codebase uses **differential drive**. We only use the maple-sim **arena** (physics world); the chassis still uses WPILib’s `DifferentialDrivetrainSim`.

## Updating maple-sim

To use a newer version (e.g. a newer release from the [releases page](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/releases)):

1. Update the version in `vendordeps/maple-sim.json` (both `"version"` and `javaDependencies[0].version`).
2. Update the version in `build.gradle` in the `implementation 'org.ironmaple:maplesim-java:...'` line.
3. Refresh Gradle.

You can also install via **WPILib → Manage Vendor Libraries → Install new libraries (online)** with:
`https://shenzhen-robotics-alliance.github.io/maple-sim/vendordep/maple-sim.json` (that may pull a different version from the docs site).

## Game pieces and projectiles (this project)

- **Arena:** maple-sim **v0.4.0-beta** uses the **2026 Rebuilt** arena and **FUEL** game pieces by default.
- **Field init:** On first simulation cycle we call `SimulatedArena.getInstance().resetFieldForAuto()` so FUEL is placed for the Rebuilt arena.
- **Logging:** Each sim cycle we log `FieldSimulation/FuelPositions` (Pose3d array) for AdvantageScope so FUEL on the field and in the air are visible.
- **Launching FUEL:** In simulation, **Driver A** runs `SimLaunchNoteCommand`: it creates a `RebuiltFuelOnFly` using current robot pose, chassis speeds, and shooter target RPM, then adds it to the arena. FUEL that hits the ground becomes `RebuiltFuelOnField` (via `enableBecomesGamePieceOnFieldAfterTouchGround()`).

## Intake simulation

**Intake sim is not wired in this project.** Maple-sim’s `IntakeSimulation` must be attached to an `AbstractDriveTrainSimulation` (a chassis body in the arena). The library only provides `SwerveDriveSimulation`; this robot uses differential drive and keeps the chassis in WPILib’s `DifferentialDrivetrainSim`, not in the maple-sim world. To add intake sim you would need to add a chassis body to the arena (e.g. a minimal subclass of `AbstractDriveTrainSimulation` that follows odometry and implements `simulationSubTick()`) and then create an `IntakeSimulation` attached to it.

## Going further with maple-sim

- **Docs:** [maple-sim documentation](https://shenzhen-robotics-alliance.github.io/maple-sim/).  
- **Game pieces / intake:** [Simulating Intake](https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-intake/), [Using the Simulated Arena](https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/), [Simulating Projectiles](https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-projectiles/).  
- **2026 Rebuilt:** Vision, PathPlanner, constants, and maple-sim (v0.4.0-beta) all use the **2026 Rebuilt** field and **FUEL** game pieces. Default `SimulatedArena.getInstance()` is the Rebuilt arena.
