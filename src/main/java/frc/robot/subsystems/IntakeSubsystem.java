package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/** Subsistema sencillo de intake usando un único SparkMax (brushless). */
public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intake = new SparkMax(DriveConstants.kIntakeMotorPort, MotorType.kBrushless);

  @SuppressWarnings("deprecation")
  public IntakeSubsystem() {
    // Configurar inversión si es necesario
    m_intake.setInverted(false);
    // Configuracion segura para el motor del intake
    try {
      com.revrobotics.spark.config.SparkMaxConfig cfg = new com.revrobotics.spark.config.SparkMaxConfig();
      cfg.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
      cfg.smartCurrentLimit(30);
      cfg.openLoopRampRate(0.2);
      m_intake.configure(cfg, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
      } catch (RuntimeException e) {
  edu.wpi.first.wpilibj.DataLogManager.log("[IntakeSubsystem] SparkMax configure failed: " + e.toString());
    }
  }

  /** Ejecuta el motor del intake con un porcentaje solicitado (-1..1). */
  public void run(double percent) {
    m_intake.set(percent);
  }

  /** Detiene el motor del intake. */
  public void stop() {
    m_intake.stopMotor();
  }

  @Override
  public void periodic() {
    // nothing
  }

  /** Devuelve el último setpoint solicitado para el motor del intake (-1..1). */
  public double getSetpoint() {
    return m_intake.get();
  }

  /** Devuelve la posición del encoder (rotaciones) del motor del intake. */
  public double getEncoderPosition() {
    return m_intake.getEncoder().getPosition();
  }

  /** Devuelve la velocidad del encoder del motor del intake. */
  public double getEncoderVelocity() {
    return m_intake.getEncoder().getVelocity();
  }

  /** Devuelve la corriente de salida (amperios) del motor del intake. */
  public double getOutputCurrent() {
    return m_intake.getOutputCurrent();
  }
}
