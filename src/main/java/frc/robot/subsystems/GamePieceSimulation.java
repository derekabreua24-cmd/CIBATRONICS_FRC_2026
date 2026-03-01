package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.DriveConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * Simulation-only subsystem: models game pieces (balls) with physics so you can test intake and
 * shooter in AdvantageScope. Balls can be picked up when the intake is running and the robot is
 * over them; balls are launched when the shooter is at speed (and carried count &gt; 0). Logs ball
 * positions for the 3D Field / game piece visualization.
 */
public class GamePieceSimulation extends SubsystemBase {

  private static final double kDtSec = 0.02;

  private final DriveSubsystem m_drive;
  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;

  /** Ball state: [0]=x, [1]=y, [2]=vx, [3]=vy (m, m/s). */
  private final List<double[]> m_balls = new ArrayList<>();
  private int m_carriedCount = 0;
  private boolean m_initialSpawnDone = false;
  private double m_lastShotTime = 0.0;

  /** Initial ball positions (x, y) in meters — spread across field for realistic intake practice. */
  private static final double[][] kInitialBalls = {
      {-5.0, 0.0}, {-2.5, 2.0}, {0.0, -2.5}, {2.5, 1.5}, {5.0, -1.0}
  };

  public GamePieceSimulation(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter) {
    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
  }

  @Override
  public void simulationPeriodic() {
    if (!RobotBase.isSimulation()) return;

    // One-time spawn of initial balls
    if (!m_initialSpawnDone) {
      for (double[] pos : kInitialBalls) {
        m_balls.add(new double[] { pos[0], pos[1], 0.0, 0.0 });
      }
      m_initialSpawnDone = true;
    }

    Pose2d robotPose = m_drive.getPose();
    double rx = robotPose.getX();
    double ry = robotPose.getY();
    double heading = robotPose.getRotation().getRadians();
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);

    // Intake zone: circle ahead of robot
    double ix = rx + cos * DriveConstants.kSimIntakeOffsetXMeters;
    double iy = ry + sin * DriveConstants.kSimIntakeOffsetXMeters;
    double intakeRadiusSq = DriveConstants.kSimBallPickupRadiusMeters
        * DriveConstants.kSimBallPickupRadiusMeters;
    boolean intakeRunning = Math.abs(m_intake.getSetpoint()) > 0.05;

    // Pick up balls in intake zone
    m_balls.removeIf(ball -> {
      if (!intakeRunning) return false;
      double dx = ball[0] - ix;
      double dy = ball[1] - iy;
      if (dx * dx + dy * dy <= intakeRadiusSq) {
        m_carriedCount++;
        return true;
      }
      return false;
    });

    // Shooter: when at speed and we have carried balls, launch one after cooldown
    double now = Timer.getFPGATimestamp();
    double rpm = Math.abs(m_shooter.getAverageVelocity());
    if (m_carriedCount > 0
        && rpm >= DriveConstants.kSimShooterRpmThreshold
        && (now - m_lastShotTime) >= DriveConstants.kSimShotCooldownSec) {
      double sx = rx + cos * DriveConstants.kSimShooterOffsetXMeters;
      double sy = ry + sin * DriveConstants.kSimShooterOffsetXMeters;
      double vx = cos * DriveConstants.kSimShooterLaunchSpeedMetersPerSec;
      double vy = sin * DriveConstants.kSimShooterLaunchSpeedMetersPerSec;
      m_balls.add(new double[] { sx, sy, vx, vy });
      m_carriedCount--;
      m_lastShotTime = now;
    }

    // Update ball physics (gravity, bounds)
    double g = DriveConstants.kSimGravityMetersPerSecSq;
    double xMin = -DriveConstants.kSimFieldHalfLengthX;
    double xMax = DriveConstants.kSimFieldHalfLengthX;
    double yMin = -DriveConstants.kSimFieldHalfWidthY;
    double yMax = DriveConstants.kSimFieldHalfWidthY;

    m_balls.removeIf(ball -> {
      ball[0] += ball[2] * kDtSec;
      ball[1] += ball[3] * kDtSec;
      ball[3] -= g * kDtSec;
      return ball[0] < xMin || ball[0] > xMax || ball[1] < yMin || ball[1] > yMax;
    });

    // Log for AdvantageScope (game piece positions)
    Pose2d[] poses = new Pose2d[m_balls.size()];
    for (int i = 0; i < m_balls.size(); i++) {
      double[] b = m_balls.get(i);
      poses[i] = new Pose2d(new Translation2d(b[0], b[1]), new Rotation2d());
    }
    Logger.recordOutput("Sim/Balls", poses);
    Logger.recordOutput("Sim/CarriedCount", m_carriedCount);
  }
}
