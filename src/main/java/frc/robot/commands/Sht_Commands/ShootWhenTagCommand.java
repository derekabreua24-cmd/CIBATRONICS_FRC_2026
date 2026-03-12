package frc.robot.commands.Sht_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 * Distance-based shooting when the camera sees any of the given AprilTag IDs.
 * Shooter spins up and feeder runs immediately. When no target tag is seen, shooter and intake stop.
 */
public class ShootWhenTagCommand extends Command {

  private final VisionSubsystem m_vision;
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;
  private final Set<Integer> m_tagIds;

  /**
   * @param vision   vision subsystem (read-only)
   * @param shooter  shooter subsystem
   * @param intake   intake subsystem
   * @param tagIds   AprilTag IDs that trigger shooting (e.g. 3, 4)
   */
  public ShootWhenTagCommand(
      VisionSubsystem vision,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      int... tagIds) {
    m_vision = vision;
    m_shooter = shooter;
    m_intake = intake;
    m_tagIds = IntStream.of(tagIds).boxed().collect(Collectors.toSet());
    addRequirements(shooter, intake);
  }

  private boolean isAnyTargetTagSeen() {
    if (m_vision == null) return false;
    for (int id : m_tagIds) {
      if (m_vision.hasSeenTag(id)) return true;
    }
    return false;
  }

  @Override
  public void initialize() {
    m_shooter.setShootReversed(true);
  }

  @Override
  public void execute() {
    if (isAnyTargetTagSeen()) {
      if (m_vision.getLastTargetDistanceMeters().isPresent()) {
        m_shooter.setVelocitySetpointFromDistanceMeters(m_vision.getLastTargetDistanceMeters().getAsDouble());
      } else {
        m_shooter.setVelocitySetpointRpm(ShooterConstants.kShooterDefaultRpm);
      }
      m_intake.runAtVoltage(ShooterConstants.kFeederVoltageDuringShoot);
    } else {
      m_shooter.stop();
      m_intake.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setShootReversed(false);
    m_shooter.stop();
    m_intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
