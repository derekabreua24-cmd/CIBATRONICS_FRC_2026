package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Subsistema simple de modo operador. Mantiene un toggle de 'operator mode' que otros
 * subsistemas pueden consultar. Proporciona un indicador en Shuffleboard y registra
 * cambios mediante DataLogManager.
 */
public class OperatorSubsystem extends SubsystemBase {
  private volatile boolean m_operatorMode = false;
  private final GenericEntry m_modeEntry;

  public OperatorSubsystem() {
    var tab = Shuffleboard.getTab("Telemetry");
    var layout = tab.getLayout("Operator", BuiltInLayouts.kList).withSize(2, 1);
  m_modeEntry = layout.add("Operator Mode", false).getEntry();
    m_modeEntry.setBoolean(m_operatorMode);
  }

  /** Alterna el estado del modo operador. */
  public void toggleMode() {
    m_operatorMode = !m_operatorMode;
    m_modeEntry.setBoolean(m_operatorMode);
    DataLogManager.log("OperatorMode=" + m_operatorMode);
  }

  /** Devuelve si el modo operador está activado actualmente. */
  public boolean isOperatorMode() {
    return m_operatorMode;
  }

  @Override
  public void periodic() {
    // No hay operaciones costosas aquí; el indicador ya se actualiza en toggle
  }
}
