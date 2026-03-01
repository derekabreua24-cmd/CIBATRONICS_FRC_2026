package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.IntakeConstants;

/**
 * Secuencia coordinada de disparo: iniciar lanzador e intake, esperar a que el lanzador
 * alcance la velocidad objetivo, y entonces activar el indexer para alimentar las bolas.
 * Si visión está disponible, usa RPM por distancia al tag; si no, usa RPM fija por defecto.
 */
public class ShootSequenceCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;
  private final VisionSubsystem m_vision;

  private static final double kRpmTolerance = 200.0;
  private long startFeedTime = 0;
  private boolean feeding = false;

  public ShootSequenceCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
    this(shooter, intake, null);
  }

  public ShootSequenceCommand(ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision) {
    m_shooter = shooter;
    m_intake = intake;
    m_vision = vision;
    addRequirements(shooter, intake);
  }

  @Override
  public void initialize() {
    updateShooterSetpoint();
    m_intake.run(IntakeConstants.kIntakeSpeed);
  }

  private void updateShooterSetpoint() {
    if (m_vision != null && m_vision.getLastTargetDistanceMeters().isPresent()) {
      m_shooter.setVelocitySetpointFromDistanceMeters(m_vision.getLastTargetDistanceMeters().getAsDouble());
    } else {
      double defaultRpm = ShooterConstants.kShooterMaxRPM * Math.abs(ShooterConstants.kShooterSpeed);
      m_shooter.setVelocitySetpointRpm(defaultRpm);
    }
  }

  @Override
  public void execute() {
    updateShooterSetpoint();

    var tuning = edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("Tuning");
    double pulse = tuning.getEntry("FeedPulseSec").getDouble(0.5);
    double pause = tuning.getEntry("FeedPauseSec").getDouble(0.2);
    boolean continuous = tuning.getEntry("FeedContinuous").getBoolean(true);

    double targetRpm = m_shooter.getTargetRpm();
    double currentRpm = Math.abs(m_shooter.getAverageVelocity());
    if (Math.abs(currentRpm - targetRpm) <= kRpmTolerance) {
      // Si el lanzador esta a velocidad: alimentar segun el modo seleccionado
  // El indexer fue eliminado; mantener el intake en marcha (se inició en initialize()).
  // En modo pulso se mantienen temporizadores internos pero no hay llamadas al motor del indexer.
      if (!continuous) {
        long now = System.currentTimeMillis();
        if (startFeedTime == 0) {
          startFeedTime = now;
          feeding = true;
        } else {
          long elapsed = now - startFeedTime;
          if (feeding && elapsed >= (long) (pulse * 1000)) {
            feeding = false;
            startFeedTime = now;
          } else if (!feeding && elapsed >= (long) (pause * 1000)) {
            feeding = true;
            startFeedTime = now;
          }
        }
      }
    } else {
      // No esta a velocidad: resetear temporizador de pulsos
      startFeedTime = 0;
      feeding = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_intake.stop();
    // Registro deshabilitado aquí (AdvantageKit se usa en otros sitios)
  }

  @Override
  public boolean isFinished() {
    // Diseñado para mantenerse pulsado por el operador; no termina por sí solo.
    return false;
  }
}
