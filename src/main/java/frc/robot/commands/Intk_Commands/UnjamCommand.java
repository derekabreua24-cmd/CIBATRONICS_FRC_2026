package frc.robot.commands.Intk_Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Unjam: intake alternates forwards and backwards at 13 V, about 7 times over 2 seconds.
 * Single tap, no hold.
 */
public class UnjamCommand extends Command {
  private static final double kUnjamVoltage = 13.0;
  private static final double kTotalDurationSec = 2.0;
  private static final int kNumPhases = 7;

  private final IntakeSubsystem m_intake;
  private final Timer m_timer = new Timer();

  public UnjamCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    double t = m_timer.get();
    double phaseDuration = kTotalDurationSec / kNumPhases;
    int phase = (int) (t / phaseDuration);
    if (phase >= kNumPhases) {
      m_intake.runAtVoltage(0.0);
      return;
    }
    boolean reverse = (phase % 2 == 0);
    m_intake.runAtVoltage(reverse ? -kUnjamVoltage : kUnjamVoltage);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(kTotalDurationSec);
  }
}
