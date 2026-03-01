package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.DriveConstants;

/**
 * Secuencia coordinada de disparo: iniciar lanzador e intake, esperar a que el lanzador
 * alcance la velocidad objetivo, y entonces activar el indexer para alimentar las bolas.
 * Disenado para ejecutarse mientras el operador mantiene pulsado el boton.
 */
public class ShootSequenceCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;

  // Tolerancia (RPM) para considerar que el lanzador está "a velocidad"
  private static final double kRpmTolerance = 200.0;
  // Estado de pulsado del alimentador
  private long startFeedTime = 0;
  private boolean feeding = false;

  public ShootSequenceCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
    m_shooter = shooter;
    m_intake = intake;
    addRequirements(shooter, intake);
  }

  @Override
  public void initialize() {
    // Iniciar inmediatamente lanzador e intake
    // Solicitar al subsistema de disparo la velocidad objetivo (RPM) calculada desde el setpoint porcentual
    double targetRpm = DriveConstants.kShooterMaxRPM * Math.abs(DriveConstants.kShooterSpeed);
    m_shooter.setVelocitySetpointRpm(targetRpm);
  m_intake.run(DriveConstants.kIntakeSpeed);
  }

  @Override
  public void execute() {
  // Mantener la velocidad objetivo del lanzador usando PID+Feedforward
    double expectedRpm = DriveConstants.kShooterMaxRPM * Math.abs(DriveConstants.kShooterSpeed);
    m_shooter.setVelocitySetpointRpm(expectedRpm);

  // Determinar comportamiento de alimentacion leyendo entradas de tuning en NetworkTable
    var tuning = edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("Tuning");
    double pulse = tuning.getEntry("FeedPulseSec").getDouble(0.5);
    double pause = tuning.getEntry("FeedPauseSec").getDouble(0.2);
    boolean continuous = tuning.getEntry("FeedContinuous").getBoolean(true);

    double currentRpm = Math.abs(m_shooter.getAverageVelocity());
    if (Math.abs(currentRpm - expectedRpm) <= kRpmTolerance) {
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
