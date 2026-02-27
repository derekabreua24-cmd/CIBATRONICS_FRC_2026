## Configuración rápida de AdvantageScope y simulación

Este documento explica cómo verificar que el proyecto funcione en simulación y cómo conectar AdvantageScope.

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
