package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.stream.IntStream;

/**
 * Small geometry helpers used by the robot for Transform2d and array utilities.
 */
public final class GeometryUtils {
  private GeometryUtils() {}

  /** Convert an array of Pose3d to Pose2d by dropping the Z/3D rotation components. */
  public static Pose2d[] pose3dArrayToPose2dArray(Pose3d[] in) {
    if (in == null) return new Pose2d[0];
    Pose2d[] out = new Pose2d[in.length];
    for (int i = 0; i < in.length; ++i) {
      out[i] = in[i].toPose2d();
    }
    return out;
  }

  /** Apply a Transform2d to every Pose2d in an array and return a new array. */
  public static Pose2d[] applyTransform2dToArray(Pose2d[] poses, Transform2d transform) {
    if (poses == null) return new Pose2d[0];
    Pose2d[] out = new Pose2d[poses.length];
    for (int i = 0; i < poses.length; ++i) {
      out[i] = poses[i].transformBy(transform);
    }
    return out;
  }

  /** Create a simple Transform2d from x,y (meters) offset and yaw degrees. */
  public static Transform2d createTransform2d(double xMeters, double yMeters, double yawDegrees) {
    return new Transform2d(new Translation2d(xMeters, yMeters), new Rotation2d(Math.toRadians(yawDegrees)));
  }

  /** Publish a Pose2d[] to a Field2d by creating objects named baseName_index. */
  public static void publishPose2dArrayToField2d(Field2d field, String baseName, Pose2d[] poses) {
    if (field == null || poses == null) return;
    IntStream.range(0, poses.length).forEach(i -> {
      try {
        field.getObject(baseName + "_" + i).setPose(poses[i]);
      } catch (RuntimeException e) {
        // Ignore per-publish errors; Field2d may throw if used concurrently in some setups.
      }
    });
  }
}
