package frc.robot.Camera_Calibration;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.File;
import java.util.Properties;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import org.littletonrobotics.junction.Logger;

/**
 * Cargador sencillo de calibración de cámara desde un archivo properties en deploy.
 * Claves:
 *  - cameraName (string)
 *  - deviceIndex (int)
 *  - fx, fy, cx, cy (double)
 *  - tagSizeMeters (double)
 *  - opcionales: cameraTx, cameraTy, cameraTz (double, metros)
 *  - opcionales: cameraRotDegX, cameraRotDegY, cameraRotDegZ (double, grados)
 */
public final class CameraCalibrationLoader {
  private CameraCalibrationLoader() {}

  public static class Calibration {
    public final String cameraName;
    public final int deviceIndex;
    public final double fx, fy, cx, cy;
    public final double tagSizeMeters;
    public final Transform3d cameraToRobot;
    /** Optional resolution (width, height). If both > 0, UsbAprilTagProcessor uses them; else 640x480. */
    public final int resolutionWidth;
    public final int resolutionHeight;

    public Calibration(
        String cameraName,
        int deviceIndex,
        double fx,
        double fy,
        double cx,
        double cy,
        double tagSizeMeters,
        Transform3d cameraToRobot) {
      this(cameraName, deviceIndex, fx, fy, cx, cy, tagSizeMeters, cameraToRobot, 0, 0);
    }

    public Calibration(
        String cameraName,
        int deviceIndex,
        double fx,
        double fy,
        double cx,
        double cy,
        double tagSizeMeters,
        Transform3d cameraToRobot,
        int resolutionWidth,
        int resolutionHeight) {
      this.cameraName = cameraName;
      this.deviceIndex = deviceIndex;
      this.fx = fx;
      this.fy = fy;
      this.cx = cx;
      this.cy = cy;
      this.tagSizeMeters = tagSizeMeters;
      this.cameraToRobot = cameraToRobot;
      this.resolutionWidth = resolutionWidth;
      this.resolutionHeight = resolutionHeight;
    }
  }

  public static Calibration loadFromProperties(String path) {
    Properties p = new Properties();

    try {
      // Ruta correcta de deploy en RoboRIO.
      File deployDir = Filesystem.getDeployDirectory();
      File file = new File(deployDir, path);

      try (FileInputStream fis = new FileInputStream(file)) {
        p.load(fis);
      }

    } catch (IOException e) {
      throw new IllegalStateException("Error al cargar la calibración de cámara desde deploy/" + path, e);
    }

    String cameraName = p.getProperty("cameraName", "usbCam0");
    int deviceIndex = Integer.parseInt(p.getProperty("deviceIndex", "0"));

    double fx = Double.parseDouble(p.getProperty("fx", "320.0"));
    double fy = Double.parseDouble(p.getProperty("fy", "320.0"));
    // cx, cy = centro de la imagen en píxeles (por defecto 320, 240 para 640x480).
    double cx = Double.parseDouble(p.getProperty("cx", "320.0"));
    double cy = Double.parseDouble(p.getProperty("cy", "240.0"));
    double tagSize = Double.parseDouble(p.getProperty("tagSizeMeters", "0.1651")); // 2026 REBUILT 6.5 in

    double tx = Double.parseDouble(p.getProperty("cameraTx", "0.0"));
    double ty = Double.parseDouble(p.getProperty("cameraTy", "0.0"));
    double tz = Double.parseDouble(p.getProperty("cameraTz", "0.0"));

    double rotX = Double.parseDouble(p.getProperty("cameraRotDegX", "0.0"));
    double rotY = Double.parseDouble(p.getProperty("cameraRotDegY", "0.0"));
    double rotZ = Double.parseDouble(p.getProperty("cameraRotDegZ", "0.0"));

    // Validación básica de coherencia.
    if (fx <= 0 || fy <= 0) {
      throw new IllegalArgumentException("Distancias focales de cámara inválidas (fx/fy deben ser > 0)");
    }

    if (tagSize <= 0) {
      throw new IllegalArgumentException("tagSizeMeters inválido (debe ser > 0)");
    }

    int resW = 0;
    int resH = 0;
    if (p.getProperty("width") != null && p.getProperty("height") != null) {
      resW = Integer.parseInt(p.getProperty("width").trim());
      resH = Integer.parseInt(p.getProperty("height").trim());
      if (resW < 160 || resH < 120) {
        throw new IllegalArgumentException("width/height deben ser >= 160, 120 para AprilTag");
      }
    }

    Transform3d cameraToRobot = new Transform3d(
        new Translation3d(tx, ty, tz),
        new Rotation3d(
            Math.toRadians(rotX),
            Math.toRadians(rotY),
            Math.toRadians(rotZ)));

  // Registro de depuración para verificación.
  Logger.recordOutput("Camera/Calibration", String.format("Calibración de cámara cargada: name=%s device=%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f tagSize=%.4f cameraToRobot=%s", cameraName, deviceIndex, fx, fy, cx, cy, tagSize, cameraToRobot.toString()));

    return new Calibration(cameraName, deviceIndex, fx, fy, cx, cy, tagSize, cameraToRobot, resW, resH);
  }
}