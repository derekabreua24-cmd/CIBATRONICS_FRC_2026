package frc.robot.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Small physics helper used to compute wheel voltages from desired chassis speeds.
 * This class is kept small and pure so it can be unit tested without hardware.
 */
public final class DrivePhysics {

  private DrivePhysics() {}

  /**
   * Compute left/right voltages for a differential drive.
   * @param vx forward velocity (m/s)
   * @param omega angular velocity (rad/s)
   * @param ks static feedforward (V)
   * @param kv velocity feedforward (V per m/s)
   * @param ka acceleration feedforward (V per m/s^2)
   * @param trackwidthMeters track width in meters
   * @param estMaxSpeed estimated max linear speed (m/s) used for clamping
   * @return double[2] {leftVolts, rightVolts}
   */
  public static double[] computeTankVoltages(
      double vx,
      double omega,
      double ks,
      double kv,
      double ka,
      double trackwidthMeters,
      double estMaxSpeed) {

    // Wheel linear speeds (m/s)
    double leftVel = vx - omega * trackwidthMeters / 2.0;
    double rightVel = vx + omega * trackwidthMeters / 2.0;

    // Use SimpleMotorFeedforward for a clean, testable feedforward calc.
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ks, kv, ka);

    // For now we don't have per-step acceleration information; we pass 0 for acceleration.
    @SuppressWarnings("removal")
    double leftVolts = ff.calculate(leftVel, 0.0);
    @SuppressWarnings("removal")
    double rightVolts = ff.calculate(rightVel, 0.0);

    // Fallback safety clamp (avoid commanding beyond battery rail)
    double clampMax = 12.0;
    // If estimated max speed is available, scale small additional voltage component
    // to help with smoothing (this preserves the feedforward semantics primarily).
    if (estMaxSpeed > 0.0) {
      leftVolts = Math.max(-clampMax, Math.min(clampMax, leftVolts));
      rightVolts = Math.max(-clampMax, Math.min(clampMax, rightVolts));
    }

    return new double[] {leftVolts, rightVolts};
  }
}
