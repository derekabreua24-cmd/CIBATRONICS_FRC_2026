package frc.robot.commands.Sht_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Shoot: spin NEO shooter to target RPM (velocity PID) and run feeder immediately. No at-speed wait.
 */
public class ShooterCommand extends Command {

  private static final double kDefaultTargetRpm = ShooterConstants.kShooterDefaultRpm;

  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;
  private final GenericEntry m_shooterRpmEntry;

  public ShooterCommand(ShooterSubsystem shooter, GenericEntry shooterRpmEntry, IntakeSubsystem intake) {
    this(shooter, shooterRpmEntry, intake, null);
  }

  public ShooterCommand(ShooterSubsystem shooter, GenericEntry shooterRpmEntry, IntakeSubsystem intake, VisionSubsystem vision) {
    m_shooter = shooter;
    m_intake = intake;
    m_shooterRpmEntry = shooterRpmEntry;
    addRequirements(shooter, intake);
  }

  private double getTargetRpm() {
    return m_shooterRpmEntry != null ? m_shooterRpmEntry.getDouble(kDefaultTargetRpm) : kDefaultTargetRpm;
  }

  @Override
  public void initialize() {
    m_shooter.setShootReversed(true);
    m_shooter.setVelocitySetpointRpm(getTargetRpm());
    if (m_intake != null) {
      m_intake.stop();
    }
  }

  @Override
  public void execute() {
    m_shooter.setVelocitySetpointRpm(getTargetRpm());
    if (m_intake != null) {
      m_intake.runAtVoltage(ShooterConstants.kFeederVoltageDuringShoot);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setShootReversed(false);
    m_shooter.stop();
    if (m_intake != null) {
      m_intake.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
