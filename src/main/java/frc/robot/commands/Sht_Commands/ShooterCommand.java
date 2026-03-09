package frc.robot.commands.Sht_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Comando para disparar: shooter (flywheel + feeder) a 11 V y inner intake a 4 V en reversa para alimentar.
 * Mantener el botón para seguir disparando.
 */
public class ShooterCommand extends Command {
  /** Voltage (V) for shooter motor (flywheel + feeder) when shooting. */
  private static final double kShootVoltage = 11.0;
  /** Voltage (V) for inner intake in feed direction (reverse) when shooting. */
  private static final double kInnerIntakeFeedVoltage = 4.0;

  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;

  public ShooterCommand(ShooterSubsystem shooter, GenericEntry shooterRpmEntry, IntakeSubsystem intake) {
    this(shooter, shooterRpmEntry, intake, null);
  }

  public ShooterCommand(ShooterSubsystem shooter, GenericEntry shooterRpmEntry, IntakeSubsystem intake, VisionSubsystem vision) {
    m_shooter = shooter;
    m_intake = intake;
    addRequirements(shooter, intake);
  }

  @Override
  public void initialize() {
    m_shooter.setShootVoltage(kShootVoltage);
    if (m_intake != null) {
      m_intake.runAtVoltage(-kInnerIntakeFeedVoltage);
    }
  }

  @Override
  public void execute() {
    m_shooter.setShootVoltage(kShootVoltage);
    if (m_intake != null) {
      m_intake.runAtVoltage(-kInnerIntakeFeedVoltage);
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
