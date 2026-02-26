package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/** Subsistema sencillo de intake usando un único SparkMax (brushless). */
public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intake = new SparkMax(DriveConstants.kIntakeMotorPort, MotorType.kBrushed);

  @SuppressWarnings("deprecation")
  public IntakeSubsystem() {
    // Configurar inversión si es necesario
    m_intake.setInverted(false); }

  /** Ejecuta el motor del intake con un porcentaje solicitado (-1..1). */
  public void run(double kIntakeSpeed) {
    m_intake.set(kIntakeSpeed);
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
