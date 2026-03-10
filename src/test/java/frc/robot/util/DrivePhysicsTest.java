package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import org.junit.jupiter.api.Test;

/**
 * Tests drive voltage computation using official WPILib APIs only:
 * DifferentialDriveKinematics.toWheelSpeeds() and SimpleMotorFeedforward.
 */
public class DrivePhysicsTest {

  private static final double kNominalVoltage = 12.0;

  /** Same logic as DriveSubsystem.driveWithSpeeds (kinematics + feedforward + clamp). */
  private static double[] computeTankVoltagesFromChassisSpeeds(
      double vxMps,
      double omegaRadPerSec,
      double ks,
      double kv,
      double ka,
      double trackwidthMeters,
      double estMaxSpeedMps) {
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackwidthMeters);
    ChassisSpeeds speeds = new ChassisSpeeds(vxMps, 0.0, omegaRadPerSec);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    double leftMps = MathUtil.clamp(wheelSpeeds.leftMetersPerSecond, -estMaxSpeedMps, estMaxSpeedMps);
    double rightMps = MathUtil.clamp(wheelSpeeds.rightMetersPerSecond, -estMaxSpeedMps, estMaxSpeedMps);
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ks, kv, ka);
    double leftVolts = MathUtil.clamp(ff.calculate(leftMps), -kNominalVoltage, kNominalVoltage);
    double rightVolts = MathUtil.clamp(ff.calculate(rightMps), -kNominalVoltage, kNominalVoltage);
    return new double[] { leftVolts, rightVolts };
  }

  @Test
  public void testComputeTankVoltagesStraight() {
    double vx = 1.0;
    double omega = 0.0;
    double ks = 0.2;
    double kv = 1.2;
    double ka = 0.05;
    double track = 0.60;
    double estMax = 3.0;

    double[] volts = computeTankVoltagesFromChassisSpeeds(vx, omega, ks, kv, ka, track, estMax);

    assertEquals(volts[0], volts[1], 1e-9);
    assertTrue(Math.abs(volts[0]) <= kNominalVoltage);
  }

  @Test
  public void testComputeTankVoltagesTurn() {
    double vx = 0.5;
    double omega = 1.0;
    double ks = 0.2;
    double kv = 1.2;
    double ka = 0.05;
    double track = 0.60;
    double estMax = 3.0;

    double[] volts = computeTankVoltagesFromChassisSpeeds(vx, omega, ks, kv, ka, track, estMax);

    assertNotEquals(volts[0], volts[1]);
    assertTrue(Math.abs(volts[0]) <= kNominalVoltage);
    assertTrue(Math.abs(volts[1]) <= kNominalVoltage);
  }
}
