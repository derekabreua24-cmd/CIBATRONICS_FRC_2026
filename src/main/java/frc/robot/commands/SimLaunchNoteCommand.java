package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/**
 * Simulation-only: launches one FUEL projectile in the maple-sim 2026 Rebuilt arena using current
 * robot pose, chassis speeds, and shooter target RPM. No-op on real robot.
 */
public class SimLaunchNoteCommand extends InstantCommand {

  private final DriveSubsystem m_drive;
  private final OdometrySubsystem m_odometry;
  private final ShooterSubsystem m_shooter;

  /** Shooter offset from robot center (forward), in meters. */
  private static final double SHOOTER_OFFSET_X = 0.2;
  /** Initial height of FUEL when launched (m). */
  private static final double LAUNCH_HEIGHT_M = 0.45;
  /** Launch angle above horizontal (degrees). */
  private static final double LAUNCH_ANGLE_DEG = 55.0;
  /** Approx launch speed (m/s) at 6000 RPM; scale by targetRpm/6000. */
  private static final double LAUNCH_SPEED_AT_6000_RPM = 20.0;

  public SimLaunchNoteCommand(DriveSubsystem drive, OdometrySubsystem odometry, ShooterSubsystem shooter) {
    super();
    m_drive = drive;
    m_odometry = odometry;
    m_shooter = shooter;
  }

  @Override
  public void initialize() {
    if (!RobotBase.isSimulation()) {
      return;
    }
    var pose = m_odometry.getPose();
    ChassisSpeeds chassisSpeeds = m_drive.getChassisSpeeds();
    double targetRpm = m_shooter.getTargetRpm();
    if (targetRpm < 100) {
      targetRpm = 3000.0;
    }
    double launchSpeedMps = (targetRpm / 6000.0) * LAUNCH_SPEED_AT_6000_RPM;
    launchSpeedMps = Math.max(5.0, Math.min(25.0, launchSpeedMps));

    RebuiltFuelOnFly fuel = new RebuiltFuelOnFly(
        pose.getTranslation(),
        new Translation2d(SHOOTER_OFFSET_X, 0),
        chassisSpeeds,
        pose.getRotation(),
        Units.Meters.of(LAUNCH_HEIGHT_M),
        Units.MetersPerSecond.of(launchSpeedMps),
        Units.Degrees.of(LAUNCH_ANGLE_DEG));
    fuel.enableBecomesGamePieceOnFieldAfterTouchGround();
    SimulatedArena.getInstance().addGamePieceProjectile(fuel);
  }
}
