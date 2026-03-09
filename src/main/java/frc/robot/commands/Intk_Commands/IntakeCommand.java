package frc.robot.commands.Intk_Commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Runs the intake (motor CAN 5) at 8 V and the shooter as feeder while active.
 * Intake pulls fuel in; feeder runs at kShooterFeedVoltage. Releasing stops both.
 */
public class IntakeCommand extends Command {
  /** Intake motor (CAN 5) voltage (V) when running in this command. */
  private static final double kIntakeVoltage = 8.0;

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
    m_intake.runAtVoltage(-kIntakeVoltage);
    m_shooter.setFeedVoltage(ShooterConstants.kShooterFeedVoltage);
    Logger.recordOutput("Intake/Events", "[IntakeCommand] Intake (CAN 5) at " + kIntakeVoltage + " V");
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_shooter.setFeedVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
