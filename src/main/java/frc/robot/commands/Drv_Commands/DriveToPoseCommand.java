package frc.robot.commands.Drv_Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

/**
 * Comando sencillo de conduccion hasta una pose usando odometria y control proporcional.
 * No es un seguidor de trayectorias completo, pero es adecuado para movimientos autonomos simples.
 */
public class DriveToPoseCommand extends Command {
  private final DriveSubsystem m_drive;
  private final OdometrySubsystem m_odometry;
  private final Pose2d m_target;
  private final double m_maxSpeed;
  private final double m_posTolMeters;
  private final double m_angTolRad;

  // Tunable gains
  private static final double kP_LINEAR = 1.2; // m/s per meter error
  private static final double kP_ANG = 2.0; // rad/s per rad error

  public DriveToPoseCommand(
      DriveSubsystem drive,
      OdometrySubsystem odometry,
      Pose2d target,
      double maxSpeed,
      double posTolMeters,
      double angTolDeg) {
    m_drive = drive;
    m_odometry = odometry;
    m_target = target;
    m_maxSpeed = Math.abs(maxSpeed);
    m_posTolMeters = Math.abs(posTolMeters);
    m_angTolRad = Math.toRadians(Math.abs(angTolDeg));
    addRequirements(drive, odometry);
  }

  @Override
  public void initialize() {
    // No-op
  }

  @Override
  public void execute() {
    var pose = m_odometry.getPose();

    // Vector to target
    double dx = m_target.getX() - pose.getX();
    double dy = m_target.getY() - pose.getY();
    double distance = Math.hypot(dx, dy);

    // Target angle of motion
    double targetAngleRad = Math.atan2(dy, dx);
    double headingRad = pose.getRotation().getRadians();
    double headingError = targetAngleRad - headingRad;

    // === ADDED === Normalize heading error to range [-PI,PI]
    headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

    // === ADDED ===
    // Forward velocity scaled by heading alignment
    double forward = kP_LINEAR * distance * Math.cos(headingError);
    forward = Math.copySign(Math.min(Math.abs(forward), m_maxSpeed), forward);

    // === ADDED ===
    // Turn toward target direction + final rotation if needed
    double finalAngleError = m_target.getRotation().minus(pose.getRotation()).getRadians();
    finalAngleError = Math.atan2(Math.sin(finalAngleError), Math.cos(finalAngleError));

    // Rotational correction combines heading + final orientation error
    double rot = clamp(kP_ANG * (headingError + finalAngleError), -0.8, 0.8);

    m_drive.arcadeDrive(forward, rot);
  }

  @Override
  public boolean isFinished() {
    var pose = m_odometry.getPose();
    double dx = m_target.getX() - pose.getX();
    double dy = m_target.getY() - pose.getY();
    double distance = Math.hypot(dx, dy);

    double angleError = Math.abs(m_target.getRotation().minus(pose.getRotation()).getRadians());

    return distance <= m_posTolMeters && angleError <= m_angTolRad;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0.0, 0.0);
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}