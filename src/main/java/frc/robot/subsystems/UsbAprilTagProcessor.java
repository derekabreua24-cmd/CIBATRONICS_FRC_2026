package frc.robot.subsystems;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.Mat;
import edu.wpi.first.wpilibj.Timer;
import org.opencv.imgproc.Imgproc;

public class UsbAprilTagProcessor extends SubsystemBase {

  private final UsbCamera m_camera;
  private final CvSink m_sink;

  private final AprilTagDetector m_detector;
  private final AprilTagPoseEstimator m_estimator;

  private final VisionSubsystem m_vision;

  // Mats reutilizados (NO asignar en cada iteracion)
  private final Mat m_frame = new Mat();
  private final Mat m_gray = new Mat();

  private final Thread m_visionThread;
  private volatile boolean m_running = true;

  public UsbAprilTagProcessor(
      String cameraName,
      int deviceIndex,
      double tagSizeMeters,
      double fx,
      double fy,
      double cx,
      double cy,
      VisionSubsystem vision) {

    m_camera = CameraServer.startAutomaticCapture(cameraName, deviceIndex);

  // IMPORTANTE: reducir resolucion para mejorar el rendimiento
    m_camera.setResolution(320, 240);
    m_camera.setFPS(20);

    m_sink = new CvSink("UsbCvSink-" + cameraName);
    m_sink.setSource(m_camera);

    m_detector = new AprilTagDetector();
    m_detector.addFamily("tag16h5");

  // Ajustes opcionales (valores seguros por defecto para roboRIO 2)
    AprilTagDetector.Config detectorConfig = m_detector.getConfig();
    detectorConfig.numThreads = 2;       // use 2 threads
    detectorConfig.quadDecimate = 2.0f;  // downscale image internally
    detectorConfig.quadSigma = 0.0f;
    m_detector.setConfig(detectorConfig);

    AprilTagPoseEstimator.Config poseConfig =
        new AprilTagPoseEstimator.Config(tagSizeMeters, fx, fy, cx, cy);

    m_estimator = new AprilTagPoseEstimator(poseConfig);

    m_vision = vision;

  // Ejecutar vision en un hilo separado (no bloquea el bucle del robot)
    m_visionThread = new Thread(this::visionLoop);
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  private void visionLoop() {
    while (m_running && !Thread.interrupted()) {

      long time = m_sink.grabFrame(m_frame);

      if (time == 0L) {
        continue; // omitir frame invalido
      }

      Imgproc.cvtColor(m_frame, m_gray, Imgproc.COLOR_BGR2GRAY);

      AprilTagDetection[] detections = m_detector.detect(m_gray);

      if (detections == null || detections.length == 0) {
        continue;
      }

      double timestamp = Timer.getFPGATimestamp();
      for (AprilTagDetection d : detections) {

        Transform3d tagToCamera = m_estimator.estimate(d);

        // Reenviar deteccion al VisionSubsystem (con marca de tiempo)
        m_vision.processDetection(d.getId(), tagToCamera, timestamp);
      }
    }
  }

  public void stop() {
    m_running = false;

    try {
      m_visionThread.join(100);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }

    m_detector.close();
    m_frame.release();
    m_gray.release();
  }

  @Override
  public void periodic() {
    // Intencionalmente vacio.
    // La vision se ejecuta en un hilo separado para evitar bloquear el bucle del robot.
  }
}