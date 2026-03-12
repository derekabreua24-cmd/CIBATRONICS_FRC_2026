package frc.robot.commands.Intk_Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Unjam: intake alternates forwards and backwards at 13 V (7 phases), and the robot drives
 * back and forth 4 times, over kTotalDurationSec. Single tap, no hold.
 */
public class UnjamCommand extends Command {
  private static final double kUnjamVoltage = 13.0;
  private static final double kTotalDurationSec = 1.5;
  private static final int kNumPhases = 7;
  /** Drive back and forth 4 times = 8 half-cycles over kTotalDurationSec. */
  private static final int kNumDriveCycles = 4;
  private static final double kUnjamDriveSpeed = 0.8;

  private final IntakeSubsystem m_intake;
  private final DriveSubsystem m_drive;
  private final Timer m_timer = new Timer();

  public UnjamCommand(IntakeSubsystem intake, DriveSubsystem drive) {
    m_intake = intake;
    m_drive = drive;
    addRequirements(intake, drive);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    double t = m_timer.get();
    if (t >= kTotalDurationSec) {
      m_intake.runAtVoltage(0.0);
      m_drive.arcadeDrive(0.0, 0.0);
      return;
    }

    // Intake: alternate 7 phases
    double phaseDuration = kTotalDurationSec / kNumPhases;
    int phase = (int) (t / phaseDuration);
    boolean intakeReverse = (phase % 2 == 0);
    m_intake.runAtVoltage(intakeReverse ? -kUnjamVoltage : kUnjamVoltage);

    // Drive: back and forth 4 times (8 half-cycles of 0.25 s each)
    double drivePhaseDuration = kTotalDurationSec / (2 * kNumDriveCycles);
    int drivePhase = (int) (t / drivePhaseDuration);
    boolean driveForward = (drivePhase % 2 == 0);
    double fwd = driveForward ? kUnjamDriveSpeed : -kUnjamDriveSpeed;
    m_drive.arcadeDrive(fwd, 0.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_drive.arcadeDrive(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(kTotalDurationSec);
  }
}
