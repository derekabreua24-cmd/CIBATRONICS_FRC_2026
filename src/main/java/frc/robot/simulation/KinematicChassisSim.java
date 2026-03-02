package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/**
 * Chassis body in the maple-sim arena that follows our robot's odometry and chassis speeds.
 * Used so {@link org.ironmaple.simulation.IntakeSimulation} can attach to a body and collect FUEL.
 * We do not apply motor forces; pose and velocity are set from outside each period.
 */
public final class KinematicChassisSim extends AbstractDriveTrainSimulation {

  public KinematicChassisSim(DriveTrainSimulationConfig config, Pose2d initialPoseOnField) {
    super(config, initialPoseOnField);
  }

  @Override
  public void simulationSubTick() {
    // No motor forces; pose and velocity are driven by setSimulationWorldPose/setRobotSpeeds from MapleSimHandler.
  }
}
