package frc.robot.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Utilidad de física para calcular tensiones de ruedas a partir de las velocidades deseadas del chasis.
 * Se mantiene pequeña y pura para poder probarla con unit tests sin hardware.
 */
public final class DrivePhysics {

  private DrivePhysics() {}

  /**
   * Calcula las tensiones izquierda/derecha para un tren diferencial.
   * @param vx velocidad hacia adelante (m/s)
   * @param omega velocidad angular (rad/s)
   * @param ks feedforward estático (V)
   * @param kv feedforward de velocidad (V por m/s)
   * @param ka feedforward de aceleración (V por m/s²)
   * @param trackwidthMeters vía en metros
   * @param estMaxSpeed velocidad lineal máxima estimada (m/s) para limitar
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

    // Velocidades lineales de las ruedas (m/s).
    double leftVel = vx - omega * trackwidthMeters / 2.0;
    double rightVel = vx + omega * trackwidthMeters / 2.0;

    // Usar SimpleMotorFeedforward para un cálculo de feedforward limpio y testeable.
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ks, kv, ka);

    // Por ahora no hay información de aceleración por paso; se pasa 0.
    @SuppressWarnings("removal")
    double leftVolts = ff.calculate(leftVel, 0.0);
    @SuppressWarnings("removal")
    double rightVolts = ff.calculate(rightVel, 0.0);

    // Limitar siempre a la tensión de batería por seguridad (evitar ordenar más de ±12 V).
    double clampMax = 12.0;
    leftVolts = Math.max(-clampMax, Math.min(clampMax, leftVolts));
    rightVolts = Math.max(-clampMax, Math.min(clampMax, rightVolts));

    return new double[] {leftVolts, rightVolts};
  }
}
