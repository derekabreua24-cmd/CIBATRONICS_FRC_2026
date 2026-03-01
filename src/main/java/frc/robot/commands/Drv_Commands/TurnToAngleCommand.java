package frc.robot.commands.Drv_Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavXSubsystem;

/** Comando PID sencillo para girar el robot hacia un angulo objetivo (grados). */
public class TurnToAngleCommand extends Command {
  private final DriveSubsystem m_drive;
  private final NavXSubsystem m_navx;
  private final PIDController m_pid;
  private final double m_targetDeg;

  // === ADDED ===
  private static final double kMaxTurnOutput = 0.8;  // limitar velocidad de giro
  private static final double kMinTurnOutput = 0.05; // superar fricción estática
  private final NetworkTable m_tuningTable =
      NetworkTableInstance.getDefault().getTable("Tuning");

  public TurnToAngleCommand(DriveSubsystem drive, NavXSubsystem navx, double targetDegrees) {
    m_drive = drive;
    m_navx = navx;
    m_targetDeg = targetDegrees;
    m_pid = new PIDController(
        AutoConstants.kTurnP,
        AutoConstants.kTurnI,
        AutoConstants.kTurnD);
    m_pid.enableContinuousInput(-180.0, 180.0);
    m_pid.setTolerance(AutoConstants.kTurnToleranceDeg);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // === ADDED ===
    m_pid.reset(); // reiniciar estado interno integral/derivativo
  }

  @Override
  public void execute() {
    m_pid.setP(m_tuningTable.getEntry("TurnP").getDouble(AutoConstants.kTurnP));
    m_pid.setI(m_tuningTable.getEntry("TurnI").getDouble(AutoConstants.kTurnI));
    m_pid.setD(m_tuningTable.getEntry("TurnD").getDouble(AutoConstants.kTurnD));
    m_pid.setTolerance(m_tuningTable.getEntry("TurnTolDeg").getDouble(AutoConstants.kTurnToleranceDeg));

    double currentDeg = m_navx.getRotation2d().getDegrees();
    double output = m_pid.calculate(currentDeg, m_targetDeg);

    // === ADDED === Clamp output to safe range
    double rot = Math.max(-kMaxTurnOutput, Math.min(kMaxTurnOutput, output));

    // === ADDED === Apply minimum output to overcome static friction
    if (!m_pid.atSetpoint() && Math.abs(rot) < kMinTurnOutput) {
      rot = Math.copySign(kMinTurnOutput, rot);
    }

    m_drive.arcadeDrive(0.0, rot);
  }

  @Override
  public boolean isFinished() {
    return m_pid.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0.0, 0.0);

    // === ADDED ===
    m_pid.reset();
  }
}