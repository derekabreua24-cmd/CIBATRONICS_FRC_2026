/*
 * Copyright (c) 2024 REV Robotics
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

package com.revrobotics.servohub;

import com.revrobotics.sim.ServoHubSimFaultManager;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class ServoHubSim {
  private final SimDouble m_deviceVoltage;
  private final SimDouble m_deviceCurrent;
  private final SimDouble m_servoVoltage;
  private final SimInt[] m_bankPulsePeriods = new SimInt[2];
  private final ServoHub m_servoHub;
  private String m_deviceName;
  private Boolean m_enable = null;

  /**
   * Create a simulated CAN Servo Hub object. This class simulates some of the internal behavior of
   * the device. This class is not required to display to the sim GUI, but is required to interact
   * with it.
   *
   * @param servoHub The Servo Hub to simulate
   */
  public ServoHubSim(ServoHub servoHub) {
    m_deviceName = "Servo Hub" + " [" + servoHub.getDeviceId() + "]";
    SimDeviceSim servoHubSim = new SimDeviceSim(m_deviceName);

    m_deviceVoltage = servoHubSim.getDouble("Device Voltage");
    m_deviceCurrent = servoHubSim.getDouble("Device Current");
    m_servoVoltage = servoHubSim.getDouble("Servo Voltage");
    m_bankPulsePeriods[0] = servoHubSim.getInt("Bank 0-2 Pulse Period");
    m_bankPulsePeriods[1] = servoHubSim.getInt("Bank 3-5 Pulse Period");
    m_servoHub = servoHub;
  }

  /**
   * Get the simulated device voltage. This matches the value from the ServoHub.getDeviceVoltage().
   *
   * @return device voltage in volts
   */
  public double getDeviceVoltage() {
    return m_deviceVoltage.get();
  }

  /**
   * Set the simulated device voltage.
   *
   * @param voltage device voltage in volts
   */
  public void setDeviceVoltage(double voltage) {
    m_deviceVoltage.set(voltage);
  }

  /**
   * Get the simulated device current. This matches the value from the ServoHub.getDeviceCurrent().
   *
   * @return device current in amps
   */
  public double getDeviceCurrent() {
    return m_deviceCurrent.get();
  }

  /**
   * Set the simulated device current.
   *
   * @param current device current in amps
   */
  public void setDeviceCurrent(double current) {
    m_deviceCurrent.set(current);
  }

  /**
   * Get the simulated servo voltage. This matches the value from the ServoHub.getServoVoltage().
   *
   * @return servo voltage in volts
   */
  public double getServoVoltage() {
    return m_servoVoltage.get();
  }

  /**
   * Set the simulated servo voltage.
   *
   * @param voltage servo voltage in volts
   */
  public void setServoVoltage(double voltage) {
    m_servoVoltage.set(voltage);
  }

  /**
   * Get the simulated bank pulse period. This matches the value from the
   * ServoHub.getBankPulsePeriod().
   *
   * @param bank the specific bank (0-2, or 3-5)to get
   * @return pulse period (in microseconds)for the specified bank
   */
  public double getBankPulsePeriod(ServoHub.Bank bank) {
    return m_bankPulsePeriods[bank.value].get();
  }

  /**
   * Set the simulated bank pulse period
   *
   * @param bank the specific bank (0-2, or 3-5)to get
   * @param pulsePeriod_us pulse period (in microseconds)for the specified bank
   */
  public void setBankPulsePeriod(ServoHub.Bank bank, int pulsePeriod_us) {
    m_bankPulsePeriods[bank.value].set(pulsePeriod_us);
  }

  /** Enable the Servo Hub Device. */
  public void enable() {
    m_enable = true;
  }

  /** Disable the Servo Hub Device */
  public void disable() {
    m_enable = false;
  }

  /**
   * Use the driver station enable as the method to enable/disable the Servo Hub. This is the
   * default, so you do not need to call this unless you previously called enable() or disable().
   */
  public void useDriverStationEnable() {
    m_enable = null;
  }

  /**
   * Get the {@link ServoHubSimFaultManager} object associated with this Servo Hub Device. This will
   * allow you to set simulated faults on your simulated device and view the Fault Manager in the
   * Sim GUI.
   *
   * @return The {@link ServoHubSimFaultManager} object associated with this Servo Hub Device
   */
  public ServoHubSimFaultManager getFaultManager() {
    return new ServoHubSimFaultManager(m_servoHub);
  }
}
