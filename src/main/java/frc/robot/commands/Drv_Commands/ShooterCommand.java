package frc.robot.commands.Drv_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.constants.ShooterConstants;

/**
 * Comando para girar el shooter mientras se mantiene el botón.
 * Si visión está disponible y hay una distancia al tag, usa RPM por distancia (setVelocitySetpointFromDistanceMeters).
 * Si no, usa la RPM de la entrada de Shuffleboard (Tuning) o el valor por defecto.
 */
public class ShooterCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final GenericEntry m_shooterRpmEntry;
  private final VisionSubsystem m_vision;

  public ShooterCommand(ShooterSubsystem shooter, GenericEntry shooterRpmEntry) {
    this(shooter, shooterRpmEntry, null);
  }

  public ShooterCommand(ShooterSubsystem shooter, GenericEntry shooterRpmEntry, VisionSubsystem vision) {
    m_shooter = shooter;
    m_shooterRpmEntry = shooterRpmEntry;
    m_vision = vision;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    updateSetpoint();
  }

  @Override
  public void execute() {
    updateSetpoint();
  }

  private void updateSetpoint() {
    if (m_vision != null && m_vision.getLastTargetDistanceMeters().isPresent()) {
      m_shooter.setVelocitySetpointFromDistanceMeters(m_vision.getLastTargetDistanceMeters().getAsDouble());
    } else {
      double defaultRpm = ShooterConstants.kShooterMaxRPM * Math.abs(ShooterConstants.kShooterSpeed);
      double targetRpm = m_shooterRpmEntry == null ? defaultRpm : m_shooterRpmEntry.getDouble(defaultRpm);
      m_shooter.setVelocitySetpointRpm(targetRpm);
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
