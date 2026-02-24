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

package com.revrobotics.sim;

import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class ServoHubSimFaultManager {
  private SimBoolean m_regulatorPGoodFault;
  private SimBoolean m_firmwareFault;
  private SimBoolean m_hardwareFault;
  private SimBoolean m_lowBatteryFault;

  private SimBoolean m_regulatorPGoodStickyFault;
  private SimBoolean m_firmwareStickyFault;
  private SimBoolean m_hardwareStickyFault;
  private SimBoolean m_lowBatteryStickyFault;

  private SimBoolean m_brownoutWarning;
  private SimBoolean m_canWarning;
  private SimBoolean m_canBusOffWarning;
  private SimBoolean m_hasResetWarning;
  private SimBoolean m_channel0OvercurrentWarning;
  private SimBoolean m_channel1OvercurrentWarning;
  private SimBoolean m_channel2OvercurrentWarning;
  private SimBoolean m_channel3OvercurrentWarning;
  private SimBoolean m_channel4OvercurrentWarning;
  private SimBoolean m_channel5OvercurrentWarning;

  private SimBoolean m_brownoutStickyWarning;
  private SimBoolean m_canStickyWarning;
  private SimBoolean m_canBusOffStickyWarning;
  private SimBoolean m_hasResetStickyWarning;
  private SimBoolean m_channel0OvercurrentStickyWarning;
  private SimBoolean m_channel1OvercurrentStickyWarning;
  private SimBoolean m_channel2OvercurrentStickyWarning;
  private SimBoolean m_channel3OvercurrentStickyWarning;
  private SimBoolean m_channel4OvercurrentStickyWarning;
  private SimBoolean m_channel5OvercurrentStickyWarning;

  private ServoHub m_servoHub;
  private String simDeviceName;

  /**
   * Create a Fault Manager object, which allows you to get and set the status of simulated faults
   * on your simulated Servo Hub. Constructing this object will also cause the Fault Manager to
   * appear in the sim gui. The state of faults can be fetched on the original Servo Hub Object.
   *
   * @param servoHub The ServoHub associated with the Fault Manager
   */
  public ServoHubSimFaultManager(ServoHub servoHub) {
    simDeviceName = ("Servo Hub [" + servoHub.getDeviceId() + "] FAULT MANAGER");
    m_servoHub = servoHub;
    setupSimDevice();
  }

  // internal setup helper
  private void setupSimDevice() {
    m_servoHub.createSimFaultManager();
    SimDeviceSim faultManager = new SimDeviceSim(simDeviceName);

    m_regulatorPGoodFault = faultManager.getBoolean("Regulator P Good Fault");
    m_firmwareFault = faultManager.getBoolean("Firmware Fault");
    m_hardwareFault = faultManager.getBoolean("Hardware Fault");
    m_lowBatteryFault = faultManager.getBoolean("Low Battery Fault");

    m_regulatorPGoodStickyFault = faultManager.getBoolean("Regulator P Good Sticky Fault");
    m_firmwareStickyFault = faultManager.getBoolean("Firmware Sticky Fault");
    m_hardwareStickyFault = faultManager.getBoolean("Hardware Sticky Fault");
    m_lowBatteryStickyFault = faultManager.getBoolean("Low Battery Sticky Fault");

    m_brownoutWarning = faultManager.getBoolean("Brownout Warning");
    m_canWarning = faultManager.getBoolean("CAN Warning");
    m_canBusOffWarning = faultManager.getBoolean("CAN Bus Off Warning");
    m_hasResetWarning = faultManager.getBoolean("Has Reset Warning");
    m_channel0OvercurrentWarning = faultManager.getBoolean("Channel 0 Over Current Warning");
    m_channel1OvercurrentWarning = faultManager.getBoolean("Channel 1 Over Current Warning");
    m_channel2OvercurrentWarning = faultManager.getBoolean("Channel 2 Over Current Warning");
    m_channel3OvercurrentWarning = faultManager.getBoolean("Channel 3 Over Current Warning");
    m_channel4OvercurrentWarning = faultManager.getBoolean("Channel 4 Over Current Warning");
    m_channel5OvercurrentWarning = faultManager.getBoolean("Channel 5 Over Current Warning");

    m_brownoutStickyWarning = faultManager.getBoolean("Brownout Sticky Warning");
    m_canStickyWarning = faultManager.getBoolean("CAN Sticky Warning");
    m_canBusOffStickyWarning = faultManager.getBoolean("CAN Bus Off Sticky Warning");
    m_hasResetStickyWarning = faultManager.getBoolean("Has Reset Sticky Warning");
    m_channel0OvercurrentStickyWarning =
        faultManager.getBoolean("Channel 0 Over Current Sticky Warning");
    m_channel1OvercurrentStickyWarning =
        faultManager.getBoolean("Channel 1 Over Current Sticky Warning");
    m_channel2OvercurrentStickyWarning =
        faultManager.getBoolean("Channel 2 Over Current Sticky Warning");
    m_channel3OvercurrentStickyWarning =
        faultManager.getBoolean("Channel 3 Over Current Sticky Warning");
    m_channel4OvercurrentStickyWarning =
        faultManager.getBoolean("Channel 4 Over Current Sticky Warning");
    m_channel5OvercurrentStickyWarning =
        faultManager.getBoolean("Channel 5 Over Current Sticky Warning");
  }

  /**
   * Set the state of the simulated faults of the device.
   *
   * <p>Use device.getFaults() to get the object and modify the parameters.
   *
   * @param faults a Faults object indicating the state of the faults
   */
  public void setFaults(ServoHub.Faults faults) {
    m_regulatorPGoodFault.set(faults.regulatorPowerGood);
    m_firmwareFault.set(faults.firmware);
    m_hardwareFault.set(faults.hardware);
    m_lowBatteryFault.set(faults.lowBattery);
  }

  /**
   * Set the state of the simulated sticky faults of the device.
   *
   * <p>Use device.getStickyFaults() to get the object and modify the parameters.
   *
   * @param faults a Faults object indicating the state of the sticky faults
   */
  public void setStickyFaults(ServoHub.Faults faults) {
    m_regulatorPGoodStickyFault.set(faults.regulatorPowerGood);
    m_firmwareStickyFault.set(faults.firmware);
    m_hardwareStickyFault.set(faults.hardware);
    m_lowBatteryStickyFault.set(faults.lowBattery);
  }

  /**
   * Set the state of the simulated warnings of the device.
   *
   * <p>Use device.getWarnings() to get the object and modify the parameters.
   *
   * @param warnings a Warnings object indicating the state of the warnings
   */
  public void setWarnings(ServoHub.Warnings warnings) {
    m_brownoutWarning.set(warnings.brownout);
    m_canWarning.set(warnings.canWarning);
    m_canBusOffWarning.set(warnings.canBusOff);
    m_hasResetWarning.set(warnings.hasReset);
    m_channel0OvercurrentWarning.set(warnings.channel0Overcurrent);
    m_channel1OvercurrentWarning.set(warnings.channel1Overcurrent);
    m_channel2OvercurrentWarning.set(warnings.channel2Overcurrent);
    m_channel3OvercurrentWarning.set(warnings.channel3Overcurrent);
    m_channel4OvercurrentWarning.set(warnings.channel4Overcurrent);
    m_channel5OvercurrentWarning.set(warnings.channel5Overcurrent);
  }

  /**
   * Set the state of the simulated sticky warnings of the device.
   *
   * <p>Use device.getStickyWarnings() to get the object and modify the parameters.
   *
   * @param warnings a Warnings object indicating the state of the sticky warnings
   */
  public void setStickyWarnings(ServoHub.Warnings warnings) {
    m_brownoutStickyWarning.set(warnings.brownout);
    m_canStickyWarning.set(warnings.canWarning);
    m_canBusOffStickyWarning.set(warnings.canBusOff);
    m_hasResetStickyWarning.set(warnings.hasReset);
    m_channel0OvercurrentStickyWarning.set(warnings.channel0Overcurrent);
    m_channel1OvercurrentStickyWarning.set(warnings.channel1Overcurrent);
    m_channel2OvercurrentStickyWarning.set(warnings.channel2Overcurrent);
    m_channel3OvercurrentStickyWarning.set(warnings.channel3Overcurrent);
    m_channel4OvercurrentStickyWarning.set(warnings.channel4Overcurrent);
    m_channel5OvercurrentStickyWarning.set(warnings.channel5Overcurrent);
  }
}
