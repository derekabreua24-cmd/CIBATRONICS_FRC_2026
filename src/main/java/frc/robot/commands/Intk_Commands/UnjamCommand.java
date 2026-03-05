package frc.robot.commands.Intk_Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Pulso corto de intake en reversa para desatascar bolas (un tap, sin mantener el botón).
 * Duración configurable en IntakeConstants.Unjam.kReverseDurationSec.
 */
public class UnjamCommand extends Command {
  private final IntakeSubsystem m_intake;
  private final Timer m_timer = new Timer();

  public UnjamCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_intake.runVoltage(-IntakeConstants.kIntakeMaxVoltage);
  }

  @Override
  public void execute() {
    // Mantener reversa durante la duración
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(IntakeConstants.Unjam.kReverseDurationSec);
  }
}
