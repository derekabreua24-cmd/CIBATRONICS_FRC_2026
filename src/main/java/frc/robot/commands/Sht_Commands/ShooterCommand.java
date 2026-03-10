package frc.robot.commands.Sht_Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Shoot: spin shooter to target RPM (closed-loop), wait until at speed, then 0.5 s later run intake to feed.
 * Hold button to keep shooting. Default RPM from dashboard entry or kShooterMaxRPM * kShooterSpeed.
 */
public class ShooterCommand extends Command {
  /** Voltage (V) for inner intake when feeding during shoot. */
  private static final double kInnerIntakeFeedVoltage = 12.0;

  private static final double kDefaultTargetRpm =
      ShooterConstants.kShooterMaxRPM * Math.abs(ShooterConstants.kShooterSpeed);

  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;
  private final GenericEntry m_shooterRpmEntry;
  /** WPILib Timer: delay feeder start until this period has elapsed after at-speed. */
  private final Timer m_feedDelayTimer = new Timer();
  private boolean m_feedDelayStarted = false;

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
    m_shooter.setVelocitySetpointRpm(getTargetRpm());
    m_feedDelayStarted = false;
    if (m_intake != null) {
      m_intake.stop();
    }
  }

  @Override
  public void execute() {
    m_shooter.setVelocitySetpointRpm(getTargetRpm());
    if (m_shooter.isAtSpeed()) {
      if (!m_feedDelayStarted) {
        m_feedDelayTimer.restart();
        m_feedDelayStarted = true;
      }
      if (m_feedDelayTimer.hasElapsed(ShooterConstants.kShooterFeedDelayAfterAtSpeedSec) && m_intake != null) {
        m_intake.runAtVoltage(kInnerIntakeFeedVoltage);
      } else if (m_intake != null) {
        m_intake.stop();
      }
    } else {
      m_feedDelayStarted = false;
      if (m_intake != null) {
        m_intake.stop();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
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
