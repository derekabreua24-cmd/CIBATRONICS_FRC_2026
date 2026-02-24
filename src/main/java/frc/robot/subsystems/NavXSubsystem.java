package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Envoltorio (wrapper) para NavX preparado para competicion con WPILib 2026.
 * Asegura la convencion de signo correcta (CCW positivo).
 */
public class NavXSubsystem extends SubsystemBase {

  private final AHRS m_ahrs;

  public NavXSubsystem() {
    // Try to configure NavX for MXP; fall back to SPI or default if MXP not available
    AHRS tmp = null;
    try {
      try {
        tmp = new AHRS(AHRS.NavXComType.valueOf("MXP"));
      } catch (IllegalArgumentException e) {
        // MXP not present in this AHRS build; try SPI
        tmp = new AHRS(AHRS.NavXComType.valueOf("SPI"));
      }
    } catch (RuntimeException e) {
      // Could not construct MXP or SPI variant; leave tmp as null and handle safely and log.
      edu.wpi.first.wpilibj.DataLogManager.log("NavXSubsystem: failed to initialise AHRS -> " + e.toString());
      tmp = null;
    }
    m_ahrs = tmp;
  }

  /**
   * Devuelve la orientacion (heading) en grados.
   * Garantiza CCW positivo (estandar de WPILib).
   */
  public double getHeadingDegrees() {
    if (m_ahrs == null) {
      return 0.0;
    }
    return -m_ahrs.getYaw();  // Invert for WPILib convention
  }

  public void reset() {
    if (m_ahrs != null) {
      m_ahrs.reset();
    }
  }

  public double getYaw() {
    if (m_ahrs == null) {
      return 0.0;
    }
    return m_ahrs.getYaw(); // Replace 'm_navx' with the actual NavX object in your subsystem
}

  /**
   * Devuelve la orientacion como Rotation2d.
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  /**
   * Devuelve el pitch en grados.
   */
  public double getPitch() {
    if (m_ahrs == null) {
      return 0.0;
    }
    return m_ahrs.getPitch();
  }

  /**
   * Devuelve el roll en grados.
   */
  public double getRoll() {
    if (m_ahrs == null) {
      return 0.0;
    }
    return m_ahrs.getRoll();
  }

  /**
   * Velocidad angular en grados/seg (CCW positivo).
   */
  public double getTurnRate() {
    if (m_ahrs == null) {
      return 0.0;
    }
    return -m_ahrs.getRate();
  }

  /**
   * Indica si NavX esta conectado.
   */
  public boolean isAvailable() {
    if (m_ahrs == null) {
      return false;
    }
    return m_ahrs.isConnected();
  }

  /**
   * Pone a cero la orientacion (heading).
   */
  public void zeroHeading() {
    if (m_ahrs != null) {
      m_ahrs.reset();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("NavX/Connected", isAvailable());
    SmartDashboard.putNumber("NavX/HeadingDeg", getHeadingDegrees());
    SmartDashboard.putNumber("NavX/PitchDeg", getPitch());
    SmartDashboard.putNumber("NavX/RollDeg", getRoll());
    SmartDashboard.putNumber("NavX/RateDegPerSec", getTurnRate());
  }
}