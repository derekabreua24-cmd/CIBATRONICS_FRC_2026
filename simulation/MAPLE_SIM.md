# Maple-Sim Integration (Team 5516 Iron Maple)

**[maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim)** from the Shenzhen Robotics Alliance (Team 5516 Iron Maple) adds a 2D physics world (dyn4j) to simulation: obstacles, game pieces, and arena physics.

**Simulator-related code** in this project lives in the **`frc.robot.simulation`** package:
- `MapleSimHandler` – arena update, field init, FUEL logging, vision pose injection (called from `RobotContainer.simulationPeriodic()`).
- `SimLaunchNoteCommand` – launches one FUEL projectile in sim (Driver A).

## What's in this project

- **Vendordep:** `vendordeps/maple-sim.json` – uses **v0.4.0-beta** (2026 Rebuilt; [releases](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/releases)).
- **Simulation:** `RobotContainer.simulationPeriodic()` delegates to `MapleSimHandler.simulationPeriodic()`, which updates the maple-sim arena, logs Fuel positions, and injects vision pose (only in sim).
- **Drivetrain:** This codebase uses **differential drive**. We only use the maple-sim **arena** (physics world); the chassis still uses WPILib's `DifferentialDrivetrainSim`.

## Updating maple-sim

To use a newer version (e.g. a newer release from the [releases page](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/releases)):

1. Update the version in `vendordeps/maple-sim.json` (both `"version"` and `javaDependencies[0].version`).
2. Update the version in `build.gradle` in the `implementation 'org.ironmaple:maplesim-java:...'` line.
3. Refresh Gradle.

You can also install via **WPILib → Manage Vendor Libraries → Install new libraries (online)** with:
`https://shenzhen-robotics-alliance.github.io/maple-sim/vendordep/maple-sim.json` (that may pull a different version from the docs site).

## Game pieces and projectiles (this project)

- **Arena:** maple-sim **v0.4.0-beta** uses the **2026 Rebuilt** arena and **FUEL** game pieces by default.
- **Field init:** `MapleSimHandler` calls `SimulatedArena.getInstance().resetFieldForAuto()` on the first sim cycle.
- **Logging:** Each sim cycle we log `FieldSimulation/FuelPositions` (Pose3d array) for AdvantageScope.
- **Launching FUEL:** **Driver A** runs `SimLaunchNoteCommand` (in `frc.robot.simulation`): creates `RebuiltFuelOnFly`, adds to arena; FUEL that hits the ground becomes `RebuiltFuelOnField`.

## Intake simulation

**Intake sim is not wired in this project.** Maple-sim's `IntakeSimulation` must be attached to an `AbstractDriveTrainSimulation` (a chassis body in the arena). The library only provides `SwerveDriveSimulation`; this robot uses differential drive. To add intake sim you would need a chassis body in the arena and an `IntakeSimulation` attached to it.

## Going further with maple-sim

- **Docs:** [maple-sim documentation](https://shenzhen-robotics-alliance.github.io/maple-sim/).
- **Game pieces / intake:** [Simulating Intake](https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-intake/), [Using the Simulated Arena](https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/), [Simulating Projectiles](https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-projectiles/).
- **2026 Rebuilt:** Vision, PathPlanner, constants, and maple-sim (v0.4.0-beta) all use the **2026 Rebuilt** field and **FUEL** game pieces.
