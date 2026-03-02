package frc.robot.commands.Log_Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TelemetrySubsystem;

/** Registra un mensaje corto de evento en el EventLog del TelemetrySubsystem. */
public class LogEventCommand extends InstantCommand {
  public LogEventCommand(TelemetrySubsystem telemetry, String message) {
    super(() -> telemetry.logEvent(message), telemetry);
  }
}
