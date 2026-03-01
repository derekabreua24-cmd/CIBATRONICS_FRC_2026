package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.stream.IntStream;

/**
 * Utilidades de geometría para Transform2d y arrays usadas por el robot.
 */
public final class GeometryUtils {
  private GeometryUtils() {}

  /** Convierte un array de Pose3d a Pose2d eliminando la componente Z y la rotación 3D. */
  public static Pose2d[] pose3dArrayToPose2dArray(Pose3d[] in) {
    if (in == null) return new Pose2d[0];
    Pose2d[] out = new Pose2d[in.length];
    for (int i = 0; i < in.length; ++i) {
      out[i] = in[i].toPose2d();
    }
    return out;
  }

  /** Aplica una Transform2d a cada Pose2d del array y devuelve un array nuevo. */
  public static Pose2d[] applyTransform2dToArray(Pose2d[] poses, Transform2d transform) {
    if (poses == null) return new Pose2d[0];
    Pose2d[] out = new Pose2d[poses.length];
    for (int i = 0; i < poses.length; ++i) {
      out[i] = poses[i].transformBy(transform);
    }
    return out;
  }

  /** Crea una Transform2d sencilla a partir del desplazamiento x,y (metros) y el yaw en grados. */
  public static Transform2d createTransform2d(double xMeters, double yMeters, double yawDegrees) {
    return new Transform2d(new Translation2d(xMeters, yMeters), new Rotation2d(Math.toRadians(yawDegrees)));
  }

  /** Publica un Pose2d[] en un Field2d creando objetos con nombre baseName_índice. */
  public static void publishPose2dArrayToField2d(Field2d field, String baseName, Pose2d[] poses) {
    if (field == null || poses == null) return;
    IntStream.range(0, poses.length).forEach(i -> {
      try {
        field.getObject(baseName + "_" + i).setPose(poses[i]);
      } catch (RuntimeException e) {
        // Ignorar errores por publicación; Field2d puede lanzar si se usa en concurrencia en algunos entornos.
      }
    });
  }
}
