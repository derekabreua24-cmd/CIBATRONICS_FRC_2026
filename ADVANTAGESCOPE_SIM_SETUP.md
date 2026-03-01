## Configuración rápida de AdvantageScope y simulación

Este documento explica cómo verificar que el proyecto funcione en simulación y cómo conectar AdvantageScope.

---

## Verification: Robot Works Correctly in AdvantageScope Simulation

The following has been verified so that the robot behaves correctly when running in desktop simulation and viewed in AdvantageScope (live or replay).

| Item | Status |
|------|--------|
| **Logger + NT4** | `Robot.robotInit()` calls `Logger.addDataReceiver(new NT4Publisher())` and `Logger.start()` so AdvantageScope can connect via "Connect to Simulator" (127.0.0.1). |
| **Drivetrain metadata** | `Robot/Drivetrain/Type`, `TrackwidthMeters`, `WheelDiameterMeters`, `GearRatio` are logged for correct field scaling. |
| **Robot pose (2D Field)** | `DriveSubsystem.periodic()` logs `Odometry/Robot` as a `Pose2d` (struct format). In AdvantageScope, open the **2D Field** tab, drag **Odometry/Robot** into the "Poses" section to see the robot on the field. |
| **Robot pose (3D Field)** | The same `Odometry/Robot` (`Pose2d`) works in the **3D Field** tab: AdvantageScope accepts 2D poses and shows the robot at ground level (Z=0). Open **3D Field** → drag **Odometry/Robot** into Poses to see the robot in 3D; use Orbit Field / Orbit Robot / Driver Station cameras as needed. |
| **Sim gyro** | In simulation, `OdometrySubsystem` uses `DriveSubsystem.getSimHeading()` instead of NavX, so pose updates correctly from `DifferentialDrivetrainSim` and the robot moves/turns as expected on the field view. |
| **NavX in sim** | NavX may fail to initialize on desktop (no MXP); code uses sim heading when `RobotBase.isSimulation()` is true, so no dependency on real hardware. |
| **Vision in sim** | Vision init failures (e.g. native/missing class) set `m_visionSubsystem = null`; `TelemetrySubsystem` and bindings null-check vision everywhere. |
| **Autonomous in sim** | `simulationInit()` calls `autonomousInit()`; if no auto is selected, `getAutonomousCommand()` returns null and no command is scheduled—robot still runs and logs. |

**Quick check:** Run "Simulate Robot", enable Teleop or Autonomous, open AdvantageScope → **File → Connect to Simulator → Default**. In the **2D Field** or **3D Field** tab, add **Odometry/Robot** to Poses; you should see the robot pose update as you drive. The 3D Field tab uses the same pose data and shows the robot on the 3D field (e.g. Orbit Robot camera to follow the robot).

---

1) Verifique las dependencias de AdvantageKit
   - El proyecto incluye `akit-java` y `akit-wpilibio` en vendordeps. Estas bibliotecas proveen los binarios nativos necesarios para la simulación de escritorio.
   - Asegúrese de que esas dependencias estén en el classpath de runtime; la salida de Gradle (`./gradlew dependencies`) debería listarlas en `runtimeClasspath` y `nativeDebug`.

2) Inicie la simulación
   - Desde VS Code use el plugin WPILib: "Simulate Robot" o ejecute la tarea Gradle `simulateRobot`.
   - El proyecto publica claves del `Logger` y valores de AdvantageKit en `Robot.simulationInit()` para facilitar la detección por AdvantageScope.

3) Abra AdvantageScope y conéctese
   - Inicie AdvantageScope en la misma máquina o en una que pueda alcanzar la dirección del simulador.
   - Si la simulación corre localmente, usar `localhost` suele funcionar.
   - Busque las entradas del `Logger` y los topics NT4 que comienzan con `akit/` o las claves que se han publicado (ej. `Smoke/AdvantageKit/Alive`).

4) Prueba rápida (smoke test)
   - Con la simulación en marcha, active Teleop o Autonomous desde la UI de simulación o seleccione un auto en Shuffleboard y habilite Autonomous.
   - La simulación publicará las claves "smoke" (configuradas en `Robot.simulationInit()`), y AdvantageScope debería detectarlas automáticamente.

Consejos y solución de problemas
   - Si no aparece información en AdvantageScope, compruebe reglas de firewall y que el publisher NT4 esté activo.
   - Verifique que las vendordeps nativas están presentes (si faltan, la simulación puede no publicar datos nativos). Revise la salida de Gradle para confirmar.
   - Si usa una red diferente (ej. dos máquinas), asegúrese de que las direcciones IP y el enrutamiento de NT4 permitan la conexión.
