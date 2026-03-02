package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.Logger;
import org.ironmaple.simulation.SimulatedArena;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Handles maple-sim (Team 5516 Iron Maple) simulation periodic logic: arena update,
 * field init, FUEL position logging, and vision pose/distance injection for sim.
 * Only used when {@link edu.wpi.first.wpilibj.RobotBase#isSimulation()} is true.
 */
public final class MapleSimHandler {

  private boolean m_fieldInitialized = false;

  /**
   * Call once per simulation cycle. Updates the maple-sim arena, logs Fuel positions
   * for AdvantageScope, and injects pose/distance into vision when present.
   */
  public void simulationPeriodic(OdometrySubsystem odometry, VisionSubsystem vision) {
    if (!edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
      return;
    }
    var arena = SimulatedArena.getInstance();
    if (!m_fieldInitialized) {
      arena.resetFieldForAuto();
      m_fieldInitialized = true;
    }
    arena.simulationPeriodic();

    Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");
    Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses != null ? fuelPoses : new Pose3d[0]);

    if (vision != null) {
      Pose2d pose = odometry.getPose();
      vision.setSimulationPoseAndDistance(pose, 2.0);
    }
  }
}
