package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class DrivePhysicsTest {

  @Test
  public void testComputeTankVoltagesStraight() {
    double vx = 1.0; // 1 m/s forward
    double omega = 0.0; // no rotation
    double ks = 0.2;
    double kv = 1.2;
    double ka = 0.05;
    double track = 0.60;
    double estMax = 3.0;

    double[] volts = DrivePhysics.computeTankVoltages(vx, omega, ks, kv, ka, track, estMax);

    // For a symmetric straight speed, left and right voltages should be equal
    assertEquals(volts[0], volts[1], 1e-9);

    // Voltages should be in a safe range
    assertTrue(Math.abs(volts[0]) <= 12.0);
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

    double[] volts = DrivePhysics.computeTankVoltages(vx, omega, ks, kv, ka, track, estMax);

    // For a turn there should be a difference between left and right voltages
    assertNotEquals(volts[0], volts[1]);
    assertTrue(Math.abs(volts[0]) <= 12.0);
    assertTrue(Math.abs(volts[1]) <= 12.0);
  }
}
