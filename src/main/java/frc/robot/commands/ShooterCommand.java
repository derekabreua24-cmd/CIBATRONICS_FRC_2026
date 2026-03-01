package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.DriveConstants;

/**
 * Command to spin up the shooter to the configured speed while held. Reads the
 * desired RPM from a Shuffleboard tuning entry (GenericEntry) so it can be adjusted at runtime.
 */
public class ShooterCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final GenericEntry m_shooterRpmEntry;

  public ShooterCommand(ShooterSubsystem shooter, GenericEntry shooterRpmEntry) {
    m_shooter = shooter;
    m_shooterRpmEntry = shooterRpmEntry;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    double defaultRpm = DriveConstants.kShooterMaxRPM * Math.abs(DriveConstants.kShooterSpeed);
    double targetRpm = m_shooterRpmEntry == null ? defaultRpm : m_shooterRpmEntry.getDouble(defaultRpm);
    m_shooter.setVelocitySetpointRpm(targetRpm);
  }

  @Override
  public void execute() {
    // Allow runtime tuning: read the desired RPM from the tuning entry each execute
    if (m_shooterRpmEntry != null) {
      double rpm = m_shooterRpmEntry.getDouble(m_shooter.getTargetRpm());
      m_shooter.setVelocitySetpointRpm(rpm);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
