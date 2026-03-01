# Final code 2 — Proyecto del robot

Este proyecto es un proyecto de robot FRC en Java (WPILib) orientado a WPILib 2026. Incluye:

- Arquitectura basada en comandos (Subsystems y Commands)
- Integración con PathPlanner (PathplannerLib 2026.1.2)
- Subsistema de telemetría con Field2d y registro (logging)

Notas rápidas:

PathPlanner
-----------
- Use la aplicación PathPlanner (versión compatible 2026.1.2) para diseñar autos.
- En la app PathPlanner, use "Deploy to robot" para copiar los JSON generados al
  directorio de deploy del robot, o copie manualmente los JSONs a `src/main/deploy`
  antes de compilar/desplegar el código del robot.
- El código del robot usa `AutoBuilder.buildAutoChooser()` para poblar un selector
  en Shuffleboard llamado "PathPlanner Autos" en la pestaña Autonomous. Seleccione
  un auto en ese selector antes de habilitar el modo Autonomous.

Feedforward / caracterización del tren de rodaje
-----------------------------------------------
- Este proyecto contiene constantes de feedforward temporales en
  `frc.robot.constants.DriveConstants`: `kDriveKS`, `kDriveKV`, `kDriveKA` y
  `kDriveEstMaxSpeed`.
- Son valores de ejemplo. Para un seguimiento de trayectoria preciso, ejecute la
  caracterización de su robot con las utilidades WPILib y reemplace esos valores
  por los medidos (ks/kv/ka).
- El método `DriveSubsystem.driveWithSpeeds(...)` utiliza una utilidad `DrivePhysics`
  que aplica un cálculo `SimpleMotorFeedforward`. Reemplace las constantes y ajuste
  las ganancias del controlador de PathPlanner para mejores resultados.

Pruebas
------
- Un test unitario pequeño (`DrivePhysicsTest`) valida la utilidad de física.
  Ejecute las pruebas con Gradle:

```pwsh
./gradlew test
```

Pasos siguientes recomendados
----------------------------
- Reemplace las constantes de feedforward por los resultados de caracterización.
- Ajuste la ganancia `PPLTVController` usada por PathPlanner (el valor por
  defecto en el código es conservador).
- Opcional: agregue más pruebas unitarias y CI para compilar y validar en cada push.
