package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/** Subsistema sencillo de indexer que hace girar un único motor para avanzar las piezas del juego. */
public class IndexerSubsystem extends SubsystemBase {
  private final SparkMax m_indexer = new SparkMax(DriveConstants.kIndexerMotorPort, MotorType.kBrushless);

  @SuppressWarnings("deprecation")
  public IndexerSubsystem() {
    // Configurar inversión si es necesario
    m_indexer.setInverted(false);
    // Configuracion segura para el motor del indexer
    try {
      com.revrobotics.spark.config.SparkMaxConfig cfg = new com.revrobotics.spark.config.SparkMaxConfig();
      cfg.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
      cfg.smartCurrentLimit(20);
      cfg.openLoopRampRate(0.2);
      m_indexer.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    } catch (RuntimeException e) {
  edu.wpi.first.wpilibj.DataLogManager.log("[IndexerSubsystem] SparkMax configure failed: " + e.toString());
    }
  }

  /** Ejecuta el indexer con el porcentaje solicitado (-1..1). */
  public void run(double percent) {
    m_indexer.set(percent);
  }

  /** Detiene el motor del indexer. */
  public void stop() {
    m_indexer.stopMotor();
  }

  @Override
  public void periodic() {
    // Nada por ahora
  }

  /** Devuelve el último setpoint solicitado para el motor del indexer (-1..1). */
  public double getSetpoint() {
    return m_indexer.get();
  }

  /** Devuelve la posición del encoder (rotaciones) para el motor del indexer. */
  public double getEncoderPosition() {
    return m_indexer.getEncoder().getPosition();
  }

  /** Devuelve la velocidad del encoder para el motor del indexer. */
  public double getEncoderVelocity() {
    return m_indexer.getEncoder().getVelocity();
  }

  /** Devuelve la corriente de salida (amperios) del motor del indexer. */
  public double getOutputCurrent() {
    return m_indexer.getOutputCurrent();
  }
}
