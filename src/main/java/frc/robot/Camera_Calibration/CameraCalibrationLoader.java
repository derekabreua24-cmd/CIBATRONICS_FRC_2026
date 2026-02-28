package frc.robot.Camera_Calibration;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.File; // --- ADDED ---
import java.util.Properties;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem; // --- ADDED ---
import org.littletonrobotics.junction.Logger;

/**
 * Simple loader for camera calibration stored as a Java properties file in deploy.
 * Keys:
 *  - cameraName (string)
 *  - deviceIndex (int)
 *  - fx, fy, cx, cy (doubles)
 *  - tagSizeMeters (double)
 *  - optional cameraTx, cameraTy, cameraTz (doubles, meters)
 *  - optional cameraRotDegX, cameraRotDegY, cameraRotDegZ (doubles, degrees)
 */
public final class CameraCalibrationLoader {
  private CameraCalibrationLoader() {}

  public static class Calibration {
    public final String cameraName;
    public final int deviceIndex;
    public final double fx, fy, cx, cy;
    public final double tagSizeMeters;
    public final Transform3d cameraToRobot;

    public Calibration(
        String cameraName,
        int deviceIndex,
        double fx,
        double fy,
        double cx,
        double cy,
        double tagSizeMeters,
        Transform3d cameraToRobot) {
      this.cameraName = cameraName;
      this.deviceIndex = deviceIndex;
      this.fx = fx;
      this.fy = fy;
      this.cx = cx;
      this.cy = cy;
      this.tagSizeMeters = tagSizeMeters;
      this.cameraToRobot = cameraToRobot;
    }
  }

  public static Calibration loadFromProperties(String path) {
    Properties p = new Properties();

    try {
      // --- ADDED: Proper RoboRIO deploy path handling ---
      File deployDir = Filesystem.getDeployDirectory();
      File file = new File(deployDir, path);

      try (FileInputStream fis = new FileInputStream(file)) {
        p.load(fis);
      }

    } catch (IOException e) {
      throw new IllegalStateException("Failed to load camera calibration from deploy/" + path, e);
    }

    String cameraName = p.getProperty("cameraName", "usbCam0");
    int deviceIndex = Integer.parseInt(p.getProperty("deviceIndex", "0"));

    double fx = Double.parseDouble(p.getProperty("fx", "800.0"));
    double fy = Double.parseDouble(p.getProperty("fy", "800.0"));
    double cx = Double.parseDouble(p.getProperty("cx", "320.0"));
    double cy = Double.parseDouble(p.getProperty("cy", "240.0"));
    double tagSize = Double.parseDouble(p.getProperty("tagSizeMeters", "0.1651")); // --- UPDATED DEFAULT FOR 2026 ---

    double tx = Double.parseDouble(p.getProperty("cameraTx", "0.0"));
    double ty = Double.parseDouble(p.getProperty("cameraTy", "0.0"));
    double tz = Double.parseDouble(p.getProperty("cameraTz", "0.0"));

    double rotX = Double.parseDouble(p.getProperty("cameraRotDegX", "0.0"));
    double rotY = Double.parseDouble(p.getProperty("cameraRotDegY", "0.0"));
    double rotZ = Double.parseDouble(p.getProperty("cameraRotDegZ", "0.0"));

    // --- ADDED: Basic sanity validation ---
    if (fx <= 0 || fy <= 0) {
      throw new IllegalArgumentException("Invalid camera focal lengths (fx/fy must be > 0)");
    }

    if (tagSize <= 0) {
      throw new IllegalArgumentException("Invalid tagSizeMeters (must be > 0)");
    }

    Transform3d cameraToRobot = new Transform3d(
        new Translation3d(tx, ty, tz),
        new Rotation3d(
            Math.toRadians(rotX),
            Math.toRadians(rotY),
            Math.toRadians(rotZ)));

  // --- ADDED: Debug print for verification (use DataLogManager) ---
  Logger.recordOutput("Camera/Calibration", String.format("Loaded Camera Calibration: name=%s device=%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f tagSize=%.4f cameraToRobot=%s", cameraName, deviceIndex, fx, fy, cx, cy, tagSize, cameraToRobot.toString()));

    return new Calibration(cameraName, deviceIndex, fx, fy, cx, cy, tagSize, cameraToRobot);
  }
}