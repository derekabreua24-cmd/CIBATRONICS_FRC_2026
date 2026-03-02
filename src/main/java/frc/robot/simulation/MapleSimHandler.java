package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import org.littletonrobotics.junction.Logger;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.MapleSimConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Handles maple-sim (Shenzhen Robotics Alliance / Team 5516 Iron Maple) simulation
 * periodic logic aligned with official examples and docs.
 * <p>
 * Flow matches <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/">Using the Simulated Arena</a>:
 * {@link SimulatedArena#getInstance()} (default 2026 Rebuilt arena), {@link SimulatedArena#resetFieldForAuto()},
 * add chassis via {@link SimulatedArena#addDriveTrainSimulation}, intake via {@link IntakeSimulation#register},
 * then {@link SimulatedArena#simulationPeriodic()} each cycle. FUEL positions from {@link SimulatedArena#getGamePiecesArrayByType}("Fuel").
 * <p>
 * See also <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-intake/">Simulating Intake</a>,
 * <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-projectiles/">Simulating Projectiles</a>.
 * Only used when {@link edu.wpi.first.wpilibj.RobotBase#isSimulation()} is true (do not run on real robot).
 */
public final class MapleSimHandler {

  private static final double INTAKE_RUNNING_THRESHOLD = 0.02;

  private boolean m_fieldInitialized = false;
  private KinematicChassisSim m_chassisSim = null;
  private IntakeSimulation m_intakeSim = null;

  private static final double SHOOTER_ACTIVE_RPM_THRESHOLD = 100.0;

  /**
   * Call once per simulation cycle. Updates the maple-sim arena, syncs chassis and intake
   * with robot state, logs Fuel positions, hub poses, intake/shooter state, and injects vision pose.
   */
  public void simulationPeriodic(
      OdometrySubsystem odometry,
      VisionSubsystem vision,
      DriveSubsystem drive,
      IntakeSubsystem intake,
      ShooterSubsystem shooter) {
    if (!edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
      return;
    }
    var arena = SimulatedArena.getInstance();

    if (!m_fieldInitialized) {
      // Official pattern: reset field, then add drivetrain and intake (see using-the-simulated-arena.md).
      arena.resetFieldForAuto();
      Pose2d initialPose = odometry.getPose();
      DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
          .withRobotMass(Units.Kilograms.of(DriveConstants.kSimMassKg))
          .withBumperSize(Units.Meters.of(0.9), Units.Meters.of(0.56))
          .withTrackLengthTrackWidth(Units.Meters.of(0.5), Units.Meters.of(DriveConstants.kTrackwidthMeters));
      m_chassisSim = new KinematicChassisSim(config, initialPose);
      arena.addDriveTrainSimulation(m_chassisSim);
      m_intakeSim = IntakeSimulation.OverTheBumperIntake(
          "Fuel",
          m_chassisSim,
          Units.Meters.of(0.5),
          Units.Meters.of(0.2),
          IntakeSimulation.IntakeSide.FRONT,
          1);
      m_intakeSim.register(arena);
      m_fieldInitialized = true;
    }

    // Command chassis with desired speeds (from motor outputs); do not overwrite pose—physics drives position and collisions.
    if (m_chassisSim != null) {
      m_chassisSim.setRobotSpeeds(drive.getDesiredChassisSpeedsForSim());
    }
    if (m_intakeSim != null && intake != null) {
      // Only "collect" when intake is running forward (positive setpoint); reverse = stop intake in sim.
      if (intake.getSetpoint() > INTAKE_RUNNING_THRESHOLD) {
        m_intakeSim.startIntake();
      } else {
        m_intakeSim.stopIntake();
      }
    }

    arena.simulationPeriodic();

    // Sync drive/odometry from maple-sim physics pose (so robot stops at walls, etc.).
    if (m_chassisSim != null && drive != null) {
      drive.setSimStateFromMapleSim(m_chassisSim.getSimulatedDriveTrainPose());
    }

    // Publish each FUEL game piece as its own Pose3d so AdvantageScope Poses tab and 3D field show them at the correct locations.
    Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");
    if (fuelPoses != null) {
      Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses);
      for (int i = 0; i < fuelPoses.length; i++) {
        Logger.recordOutput("FieldSimulation/Fuel/" + i, fuelPoses[i]);
      }
    } else {
      Logger.recordOutput("FieldSimulation/FuelPositions", new Pose3d[0]);
    }

    if (m_intakeSim != null) {
      Logger.recordOutput("FieldSimulation/IntakeFuelCount", m_intakeSim.getGamePiecesAmount());
    }

    // Hub poses so AdvantageScope 3D field shows shooting targets (intake/shooting context).
    Logger.recordOutput("FieldSimulation/Goals/BlueHub", MapleSimConstants.kBlueHubPose);
    Logger.recordOutput("FieldSimulation/Goals/RedHub", MapleSimConstants.kRedHubPose);

    // Intake active (true when running forward) for visibility in AdvantageScope line graphs.
    boolean intakeActive = intake != null && intake.getSetpoint() > INTAKE_RUNNING_THRESHOLD;
    Logger.recordOutput("FieldSimulation/IntakeActive", intakeActive);

    // Shooter active and target RPM for visibility in AdvantageScope (shooting context).
    if (shooter != null) {
      double targetRpm = shooter.getTargetRpm();
      Logger.recordOutput("FieldSimulation/ShooterActive", targetRpm > SHOOTER_ACTIVE_RPM_THRESHOLD);
      Logger.recordOutput("FieldSimulation/ShooterTargetRpm", targetRpm);
    }

    if (vision != null) {
      Pose2d pose = odometry.getPose();
      vision.setSimulationPoseAndDistance(pose, 2.0);
    }
  }

  /** Resets the chassis body pose in the physics world (e.g. when odometry is reset). Called from drive's sim reset callback. */
  public void resetChassisPose(Pose2d pose) {
    if (m_chassisSim != null) {
      m_chassisSim.setSimulationWorldPose(pose);
    }
  }
}
