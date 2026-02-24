package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Secuencia coordinada de disparo: iniciar lanzador e intake, esperar a que el lanzador
 * alcance la velocidad objetivo, y entonces activar el indexer para alimentar las bolas.
 * Disenado para ejecutarse mientras el operador mantiene pulsado el boton.
 */
public class ShootSequenceCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final IndexerSubsystem m_indexer;
  private final IntakeSubsystem m_intake;

  // Tolerancia (RPM) para considerar que el lanzador está "a velocidad"
  private static final double kRpmTolerance = 200.0;
  // Estado de pulsado del alimentador
  private long startFeedTime = 0;
  private boolean feeding = false;

  public ShootSequenceCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake) {
    m_shooter = shooter;
    m_indexer = indexer;
    m_intake = intake;
    addRequirements(shooter, indexer, intake);
  }

  @Override
  public void initialize() {
    // Iniciar inmediatamente lanzador e intake
    // Solicitar al subsistema de disparo la velocidad objetivo (RPM) calculada desde el setpoint porcentual
    double targetRpm = DriveConstants.kShooterMaxRPM * Math.abs(DriveConstants.kShooterSpeed);
    m_shooter.setVelocitySetpointRpm(targetRpm);
    m_intake.run(DriveConstants.kIntakeSpeed);
    DataLogManager.log("[ShootSequence] initialize: shooter requested speed=" + DriveConstants.kShooterSpeed);
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
      if (continuous) {
        m_indexer.run(DriveConstants.kIndexerSpeed);
      } else {
        // Modo pulsado: usar un temporizador simple para alternar encendido/apagado
        long now = System.currentTimeMillis();
        if (startFeedTime == 0) {
          startFeedTime = now;
          feeding = true;
          m_indexer.run(DriveConstants.kIndexerSpeed);
        } else {
          long elapsed = now - startFeedTime;
          if (feeding && elapsed >= (long) (pulse * 1000)) {
            feeding = false;
            startFeedTime = now;
            m_indexer.stop();
          } else if (!feeding && elapsed >= (long) (pause * 1000)) {
            feeding = true;
            startFeedTime = now;
            m_indexer.run(DriveConstants.kIndexerSpeed);
          }
        }
      }
    } else {
      // No esta a velocidad: detener indexer y resetear temporizador de pulsos
      m_indexer.stop();
      startFeedTime = 0;
      feeding = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_indexer.stop();
    m_intake.stop();
    DataLogManager.log("[ShootSequence] end interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    // Designed to be held by the operator; never finishes on its own.
    return false;
  }
}
