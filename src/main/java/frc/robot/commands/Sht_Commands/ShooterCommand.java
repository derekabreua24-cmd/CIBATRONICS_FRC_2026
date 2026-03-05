package frc.robot.commands.Sht_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.IntakeConstants;

/**
 * Comando para girar el shooter mientras se mantiene el botón.
 * También corre el intake para alimentar la nota al shooter.
 * Si visión está disponible y hay una distancia al tag, usa RPM por distancia (setVelocitySetpointFromDistanceMeters).
 * Si no, usa la RPM de la entrada de Shuffleboard (Tuning) o el valor por defecto.
 */
public class ShooterCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;
  private final GenericEntry m_shooterRpmEntry;
  private final VisionSubsystem m_vision;

  public ShooterCommand(ShooterSubsystem shooter, GenericEntry shooterRpmEntry, IntakeSubsystem intake) {
    this(shooter, shooterRpmEntry, intake, null);
  }

  public ShooterCommand(ShooterSubsystem shooter, GenericEntry shooterRpmEntry, IntakeSubsystem intake, VisionSubsystem vision) {
    m_shooter = shooter;
    m_intake = intake;
    m_shooterRpmEntry = shooterRpmEntry;
    m_vision = vision;
    addRequirements(shooter, intake);
  }

  @Override
  public void initialize() {
    updateSetpoint();
    if (m_intake != null) {
      m_intake.runVoltage(IntakeConstants.kIntakeMaxVoltage);
    }
  }

  @Override
  public void execute() {
    updateSetpoint();
    if (m_intake != null) {
      m_intake.runVoltage(IntakeConstants.kIntakeMaxVoltage);
    }
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
    if (m_intake != null) {
      m_intake.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
