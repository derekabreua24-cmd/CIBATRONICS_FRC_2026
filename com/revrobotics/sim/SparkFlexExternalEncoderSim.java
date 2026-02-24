/*
 * Copyright (c) 2024-2025 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package com.revrobotics.sim;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SparkFlexExternalEncoderSim {
  private SimDouble m_position;
  private SimDouble m_velocity;
  private SimBoolean m_isInverted;
  private SimDouble m_zeroOffset;
  private SimDouble m_positionConversionFactor;
  private SimDouble m_velocityConversionFactor;
  private SparkBase m_spark;
  private String simDeviceName;

  /**
   * Create a Spark Flex External Encoder simulation object for a Spark Flex. This will allow you to
   * read/write data from the simulated sensor and view it in the Sim GUI.
   *
   * @param motor The CANSparkFlex associated with the sensor
   */
  public SparkFlexExternalEncoderSim(SparkFlex motor) {
    simDeviceName = "SPARK Flex [" + motor.getDeviceId() + "] EXTERNAL ENCODER";
    setupSimDevice();
  }

  // internal setup helper
  private boolean setupSimDevice() {
    SimDeviceSim alternateEncoderSim = new SimDeviceSim(simDeviceName);
    m_position = alternateEncoderSim.getDouble("Position");
    m_velocity = alternateEncoderSim.getDouble("Velocity");
    m_isInverted = alternateEncoderSim.getBoolean("Is Inverted");
    m_zeroOffset = alternateEncoderSim.getDouble("Zero Offset");
    m_positionConversionFactor = alternateEncoderSim.getDouble("Position Conversion Factor");
    m_velocityConversionFactor = alternateEncoderSim.getDouble("Velocity Conversion Factor");

    return m_position == null;
  }

  // internal setup helper
  private boolean checkAndSetupSimDevice() {
    if (m_position == null) {
      return setupSimDevice();
    }
    return false;
  }

  /**
   * Set the position of the sensor, after your conversion factor
   *
   * @param position the position to set
   */
  public void setPosition(double position) {
    if (checkAndSetupSimDevice()) return;
    m_position.set(position);
  }

  /**
   * Get the position of the sensor, with the conversion factor applied
   *
   * @return the position of the sensor
   */
  public double getPosition() {
    if (checkAndSetupSimDevice()) return 0;
    return m_position.get();
  }

  /**
   * Set the velocity of the sensor, after the conversion factor
   *
   * @param velocity the velocity to set
   */
  public void setVelocity(double velocity) {
    if (checkAndSetupSimDevice()) return;
    m_velocity.set(velocity);
  }

  /**
   * Get the velocity of the sensor, with the conversion factor applied
   *
   * @return the velocity of the sensor
   */
  public double getVelocity() {
    if (checkAndSetupSimDevice()) return 0;
    return m_velocity.get();
  }

  /**
   * Set the inversion state of the sensor
   *
   * @param inverted if the sensor is inverted or not
   */
  public void setInverted(boolean inverted) {
    if (checkAndSetupSimDevice()) return;
    m_isInverted.set(inverted);
  }

  /**
   * Get the inversion state of the sensor
   *
   * @return if the sensor is inverted or not
   */
  public boolean getInverted() {
    if (checkAndSetupSimDevice()) return false;
    return m_isInverted.get();
  }

  /**
   * Set the zero offset of the sensor
   *
   * @param zeroOffset the zero offset to apply
   */
  public void setZeroOffset(double zeroOffset) {
    if (checkAndSetupSimDevice()) return;
    m_zeroOffset.set(zeroOffset);
  }

  /**
   * Get the zero offset of the sensor
   *
   * @return the zero offset of the sensor
   */
  public double getZeroOffset() {
    if (checkAndSetupSimDevice()) return 0;
    return m_zeroOffset.get();
  }

  /**
   * Get the position conversion factor of the sensor (1 by default)
   *
   * @return the conversion factor
   */
  public double getPositionConversionFactor() {
    if (checkAndSetupSimDevice()) return 0;
    return m_positionConversionFactor.get();
  }

  /**
   * Get the velocity conversion factor of the sensor (1 by default)
   *
   * @return the conversion factor
   */
  public double getVelocityConversionFactor() {
    if (checkAndSetupSimDevice()) return 0;
    return m_velocityConversionFactor.get();
  }

  /**
   * Move the sensor at the input velocity
   *
   * @param velocity the velocity of the sensor
   * @param dt the time interval of the calculation
   */
  public void iterate(double velocity, double dt) {
    if (checkAndSetupSimDevice()) return;
    double velocityRPM = velocity / getVelocityConversionFactor();
    m_position.set(m_position.get() + ((velocityRPM / 60) * dt) * getPositionConversionFactor());
  }
}
