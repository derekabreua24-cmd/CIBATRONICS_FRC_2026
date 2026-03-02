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
 * Main robot class. WPILib calls these methods by mode (Disabled, Autonomous, Teleop, Test).
 * Runs the command scheduler every cycle; initializes and shuts down vision in disabled.
 * If you rename this class or package, update Main.java.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * Esta función se ejecuta cuando el robot se inicia por primera vez y debe usarse para
   * cualquier código de inicialización.
   */
  public Robot() {
    // Instanciar RobotContainer: enlaces de botones y selector de autónomo en el dashboard.
    m_robotContainer = new RobotContainer();
  }

  /** Llamado una vez cuando el robot se inicia; inicia la telemetría (AdvantageKit/NT4). */
  @Override
  public void robotInit() {
    // El Logger de AdvantageKit se inicializa y publica en NT4 para AdvantageScope.
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
    // Publicar metadatos del tren de rodaje para AdvantageKit/AdvantageScope.
    try {
      Logger.recordOutput("Robot/Drivetrain/Type", "DifferentialDrive");
      Logger.recordOutput("Robot/Drivetrain/TrackwidthMeters", frc.robot.constants.DriveConstants.kTrackwidthMeters);
      Logger.recordOutput("Robot/Drivetrain/WheelDiameterMeters", frc.robot.constants.DriveConstants.kWheelDiameterMeters);
      Logger.recordOutput("Robot/Drivetrain/GearRatio", frc.robot.constants.DriveConstants.kDriveGearRatio);
      Logger.recordOutput("Robot/Drivetrain/DriveMotors", "SparkMax x4 (left/right pairs)");
    } catch (Exception t) {
      // No es fatal si el Logger falla; registrar el error (mejor que silenciarlo).
      try {
        Logger.recordOutput("Telemetry/Errors", "[robotInit] Error al publicar metadatos del tren de rodaje: " + t.toString());
      } catch (Exception ignore) {
        // give up silently to avoid recursive logging
      }
    }
  }

  /**
   * Esta función se llama cada 20 ms, sin importar el modo. Úsela para diagnósticos u otras
   * tareas que desee ejecutar en disabled, autonomous, teleoperated y test.
   *
   * <p>Se ejecuta después de los periodic específicos de modo, pero antes de LiveWindow y la
   * actualización integrada de SmartDashboard.
   */
  @Override
  public void robotPeriodic() {
    // Ejecuta el planificador: sondeo de botones, comandos nuevos y ya programados,
    // eliminación de comandos terminados/interrumpidos y periodic() de subsistemas.
    CommandScheduler.getInstance().run();
  }

  /** Esta función se llama una vez cada vez que el robot entra en modo Disabled. */
  @Override
  public void disabledInit() {
    m_robotContainer.logEvent("Robot disabled");
    // Detener de inmediato los procesadores de visión al deshabilitar.
    m_robotContainer.shutdownVision();
  }

  @Override
  public void disabledPeriodic() {}

  /** Este autónomo ejecuta el comando autónomo seleccionado por la clase {@link RobotContainer}. */
  @Override
  public void autonomousInit() {
    m_robotContainer.logEvent("Autonomous started");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Programar el comando autónomo seleccionado.
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** Esta función se llama periódicamente durante el modo autónomo. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.logEvent("Teleop started");
    // Asegura que el autónomo deje de ejecutarse al iniciar teleop.
    // Para que el autónomo continúe hasta ser interrumpido, quite esta línea.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** Esta función se llama periódicamente durante el control por operador (teleoperado). */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    m_robotContainer.logEvent("Test started");
    // Cancelar todos los comandos al iniciar el modo de prueba.
    CommandScheduler.getInstance().cancelAll();
  }

  /** Esta función se llama periódicamente durante el modo de prueba. */
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
    // Start selected auto for quick smoke test.
    Logger.recordOutput("Telemetry/Log", "simulationInit: starting auto for smoke test.");
    autonomousInit();

    // Publicar señal mínima de AdvantageKit/Logger para verificación en AdvantageScope.
    try {
      Logger.recordOutput("Smoke/AdvantageKit/Alive", 1);
      Logger.recordOutput("Smoke/AdvantageKit/Message", "simInit");
    } catch (Throwable t) {
      try {
        Logger.recordOutput("Telemetry/Errors", "[simulationInit] Logger smoke-test failed: " + t.toString());
      } catch (Throwable ignore) {
        // Ignorar.
      }
    }
  }

  /** Esta función se llama periódicamente durante la simulación. */
  @Override
  public void simulationPeriodic() {
    m_robotContainer.simulationPeriodic();
  }
}
