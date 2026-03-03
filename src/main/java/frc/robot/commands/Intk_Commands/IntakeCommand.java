package frc.robot.commands.Intk_Commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Runs the intake and the shooter (as feeder) while active.
 * Intake motor pulls note in; shooter motor runs in reverse to feed the note into the shooter.
 * Releasing stops both. Shooter command (RT) runs the same motor forward at RPM to launch.
 */
public class IntakeCommand extends Command {
  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;

  public IntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    m_intake = intake;
    m_shooter = shooter;
    addRequirements(intake, shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.run(-IntakeConstants.kDefaultSpeed);
    m_shooter.setFeedSpeed(ShooterConstants.kShooterFeedSpeed);
    Logger.recordOutput("Intake/Events", "[IntakeCommand] Executing intake (reversed) at speed=" + IntakeConstants.kDefaultSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_shooter.setFeedSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
