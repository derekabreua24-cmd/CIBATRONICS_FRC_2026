package frc.robot.commands.Drv_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter; // === ADDED ===
import frc.robot.subsystems.DriveSubsystem;

/** Comando por defecto para conducir con un controlador Xbox (conduccion tipo arcade). */
public class DriveCommand extends Command {

  private final DriveSubsystem m_drive;
  private final CommandXboxController m_controller;

  // === ADDED ===
  // Limitadores de slew para suavizar cambios bruscos
  private final SlewRateLimiter m_fwdLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

  public DriveCommand(DriveSubsystem drive, CommandXboxController controller) {
    m_drive = drive;
    m_controller = controller;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Zonas muertas del joystick; negar rot para que palanca a la derecha = robot gira a la derecha (arcadeDrive de WPILib positivo = izquierda).
    double fwd = -MathUtil.applyDeadband(m_controller.getLeftY(), 0.05);
    double rot = -MathUtil.applyDeadband(m_controller.getRightX(), 0.05);

    // === ADDED ===
    // Squared inputs for finer control sensitivity
    fwd = Math.copySign(fwd * fwd, fwd);
    rot = Math.copySign(rot * rot, rot);

    // === ADDED ===
    // Limitación de slew rate
    fwd = m_fwdLimiter.calculate(fwd);
    rot = m_rotLimiter.calculate(rot);

    // Conduccion por arcade
    m_drive.arcadeDrive(fwd, rot);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}