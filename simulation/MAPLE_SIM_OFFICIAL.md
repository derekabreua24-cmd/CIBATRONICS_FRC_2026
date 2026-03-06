# MapleSim – Official usage reference

This project uses [maple-sim](https://shenzhen-robotics-alliance.github.io/maple-sim/) (Shenzhen Robotics Alliance / Team 5516 Iron Maple) for physics simulation. Below are the official links and how this repo follows them.

## Official documentation

- **Home:** https://shenzhen-robotics-alliance.github.io/maple-sim/
- **Using the Simulated Arena:** https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/
- **Simulating Intake:** https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-intake/
- **Simulating Projectiles:** https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-projectiles/
- **Simulation Details:** https://shenzhen-robotics-alliance.github.io/maple-sim/simulation-details/
- **Installing maple-sim:** https://shenzhen-robotics-alliance.github.io/maple-sim/installing-maple-sim/
- **Javadocs:** https://shenzhen-robotics-alliance.github.io/maple-sim/javadocs/

## How this project uses maple-sim

| Official guideline | Our implementation |
|--------------------|--------------------|
| Call `SimulatedArena.getInstance().simulationPeriodic()` in sim only | `MapleSimHandler.simulationPeriodic()` (called from `Robot.simulationPeriodic()` via `RobotContainer`), guarded by `RobotBase.isSimulation()` |
| Do not call `simulationPeriodic()` on a real robot | Guard at start of `MapleSimHandler.simulationPeriodic()` |
| Do not override simulation timings when using AdvantageKit | We do not call `overrideSimulationTimings()` |
| Use `resetFieldForAuto()` to populate field with Fuel (2026 Rebuilt) | Called once when `m_fieldInitialized` is false in `MapleSimHandler` |
| Register drivetrain with arena | `arena.addDriveTrainSimulation(m_chassisSim)` with our `KinematicChassisSim` |
| Create intake with `IntakeSimulation.OverTheBumperIntake(...)` and attach to drivetrain | Done in `MapleSimHandler` with "Fuel", width 0.5 m, extension 0.2 m, FRONT, capacity 1 |
| Register intake with arena | `m_intakeSim.register(arena)` |
| Start/stop intake via `startIntake()` / `stopIntake()` | Called each cycle from intake setpoint in `MapleSimHandler` |
| Get Fuel poses with `getGamePiecesArrayByType("Fuel")` and log for AdvantageScope | We log `FieldSimulation/FuelPositions` and `FieldSimulation/Fuel/0`, etc. |
| Launch projectile with `RebuiltFuelOnFly` and `addGamePieceProjectile()` | `SimLaunchFuelCommand` builds `RebuiltFuelOnFly` and calls `SimulatedArena.getInstance().addGamePieceProjectile(fuel)` |
| Configure projectile to become field piece on touchdown | `RebuiltFuelOnFly` constructor / `enableBecomesGamePieceOnFieldAfterTouchGround()` |

## Version and season

- **Vendordep:** `vendordeps/maple-sim.json` → `maplesim-java` **0.4.0-beta**.
- **Season:** All simulation is **2026 Rebuilt**. `SimulatedArena.getInstance()` returns `Arena2026Rebuilt`; game pieces are FUEL; hubs match RebuiltHub.
- **Install from:** https://shenzhen-robotics-alliance.github.io/maple-sim/vendordep/maple-sim.json (per official installing page).

## Code locations

- **Arena, chassis, intake, FUEL logging, goals, vision injection (2026 Rebuilt):** `frc.robot.simulation.MapleSimHandler`
- **Kinematic chassis (differential, no wheel physics):** `frc.robot.simulation.KinematicChassisSim`
- **Launch FUEL projectile (sim only, 2026 Rebuilt):** `frc.robot.simulation.SimLaunchFuelCommand`
- **Sim initial pose / hub poses (2026 Rebuilt):** `frc.robot.constants.MapleSimConstants`
