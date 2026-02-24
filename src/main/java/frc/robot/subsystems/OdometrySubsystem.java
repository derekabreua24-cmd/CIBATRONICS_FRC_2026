package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Gestor centralizado de odometría.
 *
 * Este subsistema:
 *  - Actualiza el estimador de pose del tren de rodaje usando la orientación del giroscopio
 *  - Proporciona acceso a la pose estimada actual
 *  - Permite reinicios controlados de la pose
 *  - Reenvía medidas de visión para fusión
 *
 * IMPORTANTE:
 * DriveSubsystem NO debe actualizar la odometría en su propio método periodic().
 */
public class OdometrySubsystem extends SubsystemBase {

  private final DriveSubsystem m_drive;
  private final NavXSubsystem m_navx;

  public OdometrySubsystem(DriveSubsystem drive, NavXSubsystem navx) {
    m_drive = drive;
    m_navx = navx;
  }

  @Override
  public void periodic() {
    // Actualizar el estimador una vez por bucle usando la orientacion actual del giroscopio
    Rotation2d heading = m_navx.getRotation2d();
    m_drive.updateOdometry(heading);
  }

  /** Devuelve la pose estimada actual del robot. */
  public Pose2d getPose() {
    return m_drive.getPose();
  }

  /**
   * Reinicia la odometria a una pose especificada usando la orientacion actual del giroscopio.
   * Se confia en el heading del giroscopio para la rotacion.
   */
  public void resetOdometry(Pose2d pose) {
    Rotation2d heading = m_navx.getRotation2d();
    m_drive.resetOdometry(pose, heading);
    edu.wpi.first.wpilibj.DataLogManager.log("[Odometry] Reset to pose: " + pose);
  }

  /**
   * Agrega una medida de pose basada en vision para la fusion del estimador.
   *
   * @param visionPose Pose del robot estimada por vision
   * @param timestampSeconds Marca de tiempo en segundos (tiempo FPGA)
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    m_drive.addVisionMeasurement(visionPose, timestampSeconds);
    edu.wpi.first.wpilibj.DataLogManager.log("[Odometry] Vision measurement added at t=" + timestampSeconds + " pose=" + visionPose);
  }
}