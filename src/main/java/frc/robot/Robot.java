// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

/**
 * Los métodos en esta clase son llamados automáticamente según el modo (Disabled/Auton/Teleop),
 * como se describe en la documentación de TimedRobot. Si cambia el nombre de esta clase o el
 * paquete después de crear el proyecto, también debe actualizar el archivo Main.java.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * Esta función se ejecuta cuando el robot se inicia por primera vez y debe usarse para
   * cualquier código de inicialización.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /** Llamado una vez cuando el robot se inicia; inicia la telemetría (AdvantageKit/NT4). */
  @Override
  public void robotInit() {
    // AdvantageKit Logger is initialized below and publishes to NT4 for AdvantageScope.
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
    // Publish drivetrain metadata for AdvantageKit / AdvantageScope so tools can
    // identify this robot's drive type and basic physical parameters.
    try {
      Logger.recordOutput("Robot/Drivetrain/Type", "DifferentialDrive");
      Logger.recordOutput("Robot/Drivetrain/TrackwidthMeters", Constants.DriveConstants.kTrackwidthMeters);
      Logger.recordOutput("Robot/Drivetrain/WheelDiameterMeters", Constants.DriveConstants.kWheelDiameterMeters);
      Logger.recordOutput("Robot/Drivetrain/GearRatio", Constants.DriveConstants.kDriveGearRatio);
      Logger.recordOutput("Robot/Drivetrain/DriveMotors", "SparkMax x4 (left/right pairs)");
    } catch (Throwable t) {
      // Not fatal if Logger isn't ready; attempt to record the error and otherwise continue silently.
      try {
        Logger.recordOutput("Telemetry/Errors", "[robotInit] Failed to publish drivetrain metadata to Logger: " + t.toString());
      } catch (Throwable ignore) {
        // intentionally no further fallback to avoid terminal spam
      }
    }
  }

  /**
  /**
   * Esta función se llama cada 20 ms, sin importar el modo. Úsela para diagnósticos u otras
   * tareas que desee ejecutar en disabled, autonomous, teleoperated y test.
   *
   * <p>Se ejecuta después de los periodic específicos de modo, pero antes de LiveWindow y la
   * actualización integrada de SmartDashboard.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** Esta función se llama una vez cada vez que el robot entra en modo Disabled. */
  @Override
  public void disabledInit() {
    // Ensure vision processors are stopped immediately when disabled
    m_robotContainer.shutdownVision();
  }

  @Override
  public void disabledPeriodic() {}

  /** Este autónomo ejecuta el comando autónomo seleccionado por la clase {@link RobotContainer}. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** Esta función se llama periódicamente durante el modo autónomo. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** Esta función se llama periódicamente durante el control por operador (teleoperado). */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /**
   * Llamado una vez cuando se inicia la simulación en modo escritorio.
   *
   * Esta implementación inicia automáticamente el autónomo seleccionado para
   * facilitar pruebas rápidas (smoke tests) y publica una señal mínima a
   * AdvantageKit/Logger para que AdvantageScope pueda detectar que la
   * telemetría está activa.
   */
  @Override
  public void simulationInit() {
    // Iniciar el autónomo seleccionado para pruebas rápidas del AutoBuilder/chooser
  Logger.recordOutput("Telemetry/Log", "simulationInit: auto-starting autonomous for smoke test.");
    autonomousInit();

    // Publicar una señal simple de AdvantageKit/Logger para verificación en AdvantageScope
    try {
      Logger.recordOutput("Smoke/AdvantageKit/Alive", 1);
      Logger.recordOutput("Smoke/AdvantageKit/Message", "simInit");
    } catch (Throwable t) {
      try {
        Logger.recordOutput("Telemetry/Errors", "[simulationInit] Logger smoke-test failed: " + t.toString());
      } catch (Throwable ignore) {
        // swallow
      }
    }
  }

  /** Esta función se llama periódicamente durante la simulación. */
  @Override
  public void simulationPeriodic() {}
}
