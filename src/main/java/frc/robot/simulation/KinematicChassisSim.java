package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/**
 * Chassis body in the maple-sim arena. Velocity is set each tick from desired chassis speeds;
 * the physics engine integrates position and resolves collisions (e.g. walls). Pose is read back
 * and used to update drive/odometry. Used so {@link org.ironmaple.simulation.IntakeSimulation} can attach and collect FUEL.
 */
public final class KinematicChassisSim extends AbstractDriveTrainSimulation {

  public KinematicChassisSim(DriveTrainSimulationConfig config, Pose2d initialPoseOnField) {
    super(config, initialPoseOnField);
  }

  @Override
  public void simulationSubTick() {
    // No motor forces; velocity is set by MapleSimHandler via setRobotSpeeds; pose comes from physics.
  }
}
