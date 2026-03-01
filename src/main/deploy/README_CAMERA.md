# Calibración de cámara y configuración AprilTag

Este proyecto espera un archivo properties en la ruta de deploy:

  src/main/deploy/camera/camera_calib.properties

## Cámaras USB genéricas

El código está pensado para funcionar con **cualquier cámara USB** compatible con WPILib (UVC). Por defecto se usa resolución **320×240** (buen rendimiento). **cx** y **cy** deben ser el **centro de la imagen en píxeles**: para 320×240 use **cx=160, cy=120**; para 640×480 use **cx=320, cy=240**. Si no, la estimación de pose AprilTag será incorrecta. Si su cámara solo ofrece 640×480, descomente y ajuste `width=640` y `height=480` en el properties y ponga cx=320, cy=240.

Claves (valores por defecto):
- cameraName (string) - usbCam0
- deviceIndex (int) - 0 (primer USB; use 1 para una segunda cámara)
- fx, fy (double) - distancias focales en píxeles (por defecto 320 para 320×240; calibrar para precisión)
- cx, cy (double) - **punto principal = centro de imagen** (160, 120 para 320×240; 320, 240 para 640×480)
- tagSizeMeters (double) - lado del AprilTag en metros (2026 REBUILT: 0.1651)
- width, height (int, opcional) - si se definen, se usa esta resolución en lugar de 320×240
- cameraTx/cameraTy/cameraTz (double, opcional) - traslación cámara→robot en metros
- cameraRotDegX/Y/Z (double, opcional) - rotación cámara→robot en grados

Comportamiento
- Al arrancar el robot se carga el archivo de calibración y se inician el subsistema de visión y el procesador USB AprilTag.
- El sistema es intencionadamente fail-fast: si falta o está mal formado el archivo de calibración o el JSON del layout AprilTag, el arranque lanzará una excepción (para detectar el problema pronto).
- Para detener el procesador USB al deshabilitar, `Robot.disabledInit()` llama al shutdown del contenedor.

Archivos
* Para 2026 REBUILT el detector usa la familia **tag36h11** y tamaño de tag 6.5 in (0.1651 m). El layout de campo puede cargarse desde `deploy/apriltagfield_2026.json` (o usar el predeterminado de WPILib).
- `src/main/deploy/camera/camera_calib.properties` – propiedades de calibración de ejemplo (editar antes de desplegar).

Si prefieres un arranque tolerante a fallos en lugar de fail-fast, se puede cambiar el comportamiento para registrar y continuar sin visión.
