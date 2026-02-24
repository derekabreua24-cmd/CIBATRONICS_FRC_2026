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
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SparkLimitSwitchSim {
  private SimBoolean m_isPressed;
  private SimBoolean m_isEnabled;
  private SparkBase m_spark;
  private String simDeviceName;
  private final boolean isForward;

  /**
   * Create a Limit Switch simulation object for a Spark MAX. This will allow you to read/write data
   * from the simulated sensor and view it in the Sim GUI.
   *
   * @param motor The CANSparkMax associated with the sensor
   * @param forward if the switch is forward or not
   */
  public SparkLimitSwitchSim(SparkMax motor, boolean forward) {
    simDeviceName =
        ("SPARK MAX ["
            + motor.getDeviceId()
            + "] LIMIT SWITCH ("
            + (forward ? "FORWARD" : "REVERSE")
            + ")");
    setupSimDevice();
    m_spark = motor;
    isForward = forward;
  }

  /**
   * Create a Limit Switch simulation object for a Spark Flex. This will allow you to read/write
   * data from the simulated sensor and view it in the Sim GUI.
   *
   * @param motor The CANSparkMax associated with the sensor
   * @param forward if the switch is forward or not
   */
  public SparkLimitSwitchSim(SparkFlex motor, boolean forward) {
    simDeviceName =
        ("SPARK Flex ["
            + motor.getDeviceId()
            + "] LIMIT SWITCH ("
            + (forward ? "FORWARD" : "REVERSE")
            + ")");
    setupSimDevice();
    m_spark = motor;
    isForward = forward;
  }

  // internal setup helper
  private boolean setupSimDevice() {
    SimDeviceSim limitSwitchSim = new SimDeviceSim(simDeviceName);
    m_isPressed = limitSwitchSim.getBoolean("Is Pressed");
    m_isEnabled = limitSwitchSim.getBoolean("Is Enabled");

    return m_isPressed == null;
  }

  // internal setup helper
  private boolean checkAndSetupSimDevice() {
    if (m_isPressed == null) {
      return setupSimDevice();
    }
    return false;
  }

  /**
   * Set the state of the limit switch
   *
   * @param state if the switch is pressed or not
   */
  public void setPressed(boolean state) {
    if (checkAndSetupSimDevice()) return;
    m_isPressed.set(state);
  }

  /**
   * Get the state of the limit switch
   *
   * @return if the switch is pressed or not
   */
  public boolean getPressed() {
    if (checkAndSetupSimDevice()) return false;
    return m_isPressed.get();
  }

  /**
   * Get the enabled state of the limit switch
   *
   * <p>Enable the limit switch via the original SparkLimitSwitch object
   *
   * @return if the switch is enabled or not
   */
  public boolean getEnabled() {
    if (checkAndSetupSimDevice()) return false;
    return m_isEnabled.get();
  }
}
