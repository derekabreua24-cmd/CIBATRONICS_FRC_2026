package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavXSubsystem extends SubsystemBase {

  private final AHRS m_ahrs;

  public NavXSubsystem() {
    AHRS tmp;
    try {
      // For NavX2 on RoboRIO MXP port
      tmp = new AHRS(AHRS.NavXComType.kMXP_SPI);
  Logger.recordOutput("NavX/Status", "NavX initialized on MXP SPI.");
    } catch (RuntimeException e) {
  Logger.recordOutput("NavX/Errors", "NavX failed to initialize: " + e.toString());
      tmp = null;
    }
    m_ahrs = tmp;
  }

  public Rotation2d getRotation2d() {
    if (m_ahrs == null) {
      return new Rotation2d();
    }
    return m_ahrs.getRotation2d();
  }

  public double getYaw() {
    if (m_ahrs == null) {
      return 0.0;
    }
    return m_ahrs.getYaw();
  }

  public double getTurnRate() {
    if (m_ahrs == null) {
      return 0.0;
    }
    return m_ahrs.getRate();
  }

  public void zeroHeading() {
    if (m_ahrs != null) {
      m_ahrs.zeroYaw();
    }
  }

  public boolean isConnected() {
    return m_ahrs != null && m_ahrs.isConnected();
  }

  public void reset() {
  if (m_ahrs != null) {
    m_ahrs.reset();
  }
}

public double getPitch() {
  if (m_ahrs == null) {
    return 0.0;
  }
  return m_ahrs.getPitch();
}

public double getRoll() {
  if (m_ahrs == null) {
    return 0.0;
  }
  return m_ahrs.getRoll();
}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("NavX/Connected", isConnected());
    SmartDashboard.putNumber("NavX/Yaw", getYaw());
    SmartDashboard.putNumber("NavX/TurnRate", getTurnRate());
  }
}