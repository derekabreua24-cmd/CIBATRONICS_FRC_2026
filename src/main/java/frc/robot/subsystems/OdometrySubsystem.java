package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotBase;
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
    // En simulación usar el rumbo del sim del tren de rodaje para que la pose sea correcta; en robot usar NavX (WPILib 2026).
    Rotation2d heading =
        (RobotBase.isSimulation() && m_drive.getSimHeading().isPresent())
            ? m_drive.getSimHeading().get()
            : m_navx.getRotation2d();
    m_drive.updateOdometryWithTime(Timer.getFPGATimestamp(), heading);

    Logger.recordOutput("Odometry/Update", "[Odometry] Updating odometry. pose=" + getPose());
  }

  /** Devuelve la pose estimada actual del robot. */
  public Pose2d getPose() {
    return m_drive.getPose();
  }

  /**
   * Reinicia la odometría a la pose indicada.
   * Usa la rotación de la pose (p. ej. la que envía PathPlanner al iniciar una ruta).
   */
  public void resetOdometry(Pose2d pose) {
    Rotation2d heading = pose.getRotation();
    m_drive.resetOdometry(pose, heading);
    Logger.recordOutput("Odometry/Reset", "[Odometry] Reset to pose: " + pose);
  }

  /**
   * Agrega una medida de pose basada en vision para la fusion del estimador.
   *
   * @param visionPose Pose del robot estimada por vision
   * @param timestampSeconds Marca de tiempo en segundos (tiempo FPGA)
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    m_drive.addVisionMeasurement(visionPose, timestampSeconds);
    Logger.recordOutput("Odometry/Vision", "[Odometry] Vision measurement added at t=" + timestampSeconds + " pose=" + visionPose);
  }
}