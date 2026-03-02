package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import org.littletonrobotics.junction.Logger;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Handles maple-sim (Team 5516 Iron Maple) simulation periodic logic: arena update,
 * field init, kinematic chassis (for intake), intake sim, FUEL logging, and vision injection.
 * Only used when {@link edu.wpi.first.wpilibj.RobotBase#isSimulation()} is true.
 */
public final class MapleSimHandler {

  private static final double INTAKE_RUNNING_THRESHOLD = 0.02;

  private boolean m_fieldInitialized = false;
  private KinematicChassisSim m_chassisSim = null;
  private IntakeSimulation m_intakeSim = null;

  /**
   * Call once per simulation cycle. Updates the maple-sim arena, syncs chassis and intake
   * with robot state, logs Fuel positions, and injects vision pose when present.
   */
  public void simulationPeriodic(
      OdometrySubsystem odometry,
      VisionSubsystem vision,
      DriveSubsystem drive,
      IntakeSubsystem intake) {
    if (!edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
      return;
    }
    var arena = SimulatedArena.getInstance();

    if (!m_fieldInitialized) {
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

    if (m_chassisSim != null) {
      m_chassisSim.setSimulationWorldPose(odometry.getPose());
      m_chassisSim.setRobotSpeeds(drive.getChassisSpeeds());
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

    Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");
    Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses != null ? fuelPoses : new Pose3d[0]);

    if (m_intakeSim != null) {
      Logger.recordOutput("FieldSimulation/IntakeFuelCount", m_intakeSim.getGamePiecesAmount());
    }

    if (vision != null) {
      Pose2d pose = odometry.getPose();
      vision.setSimulationPoseAndDistance(pose, 2.0);
    }
  }
}
