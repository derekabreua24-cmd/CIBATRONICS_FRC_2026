package frc.robot.commands.Sht_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Runs shooter and intake (feeds balls) only while the camera detects AprilTag ID 3.
 * When tag 3 is not seen, shooter and intake are stopped. RPM uses distance from vision when
 * available, otherwise default max RPM.
 */
public class ShootWhenTag3Command extends Command {

  private static final int kTargetTagId = 3;

  private final VisionSubsystem m_vision;
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;

  public ShootWhenTag3Command(
      VisionSubsystem vision,
      ShooterSubsystem shooter,
      IntakeSubsystem intake) {
    m_vision = vision;
    m_shooter = shooter;
    m_intake = intake;
    addRequirements(shooter, intake);
    // Vision is read-only; no addRequirements(vision)
  }

  @Override
  public void execute() {
    if (m_vision != null && m_vision.hasSeenTag(kTargetTagId)) {
      if (m_vision.getLastTargetDistanceMeters().isPresent()) {
        m_shooter.setVelocitySetpointFromDistanceMeters(m_vision.getLastTargetDistanceMeters().getAsDouble());
      } else {
        m_shooter.setVelocitySetpointRpm(ShooterConstants.kShooterMaxRPM * Math.abs(ShooterConstants.kShooterSpeed));
      }
      m_intake.runVoltage(IntakeConstants.kIntakeMaxVoltage);
    } else {
      m_shooter.stop();
      m_intake.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
