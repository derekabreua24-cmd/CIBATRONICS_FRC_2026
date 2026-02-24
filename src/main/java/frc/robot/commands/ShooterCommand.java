package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;

/** Ejecuta el lanzador a una velocidad fija mientras el comando este activo. */
public class ShooterCommand extends Command {
  private final ShooterSubsystem m_shooter;

  public ShooterCommand(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.setSpeed(DriveConstants.kShooterSpeed);
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
