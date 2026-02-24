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

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SparkSimFaultManager {
  private SimBoolean m_otherFault;
  private SimBoolean m_motorTypeFault;
  private SimBoolean m_sensorFault;
  private SimBoolean m_canFault;
  private SimBoolean m_temperatureFault;
  private SimBoolean m_drvFault;
  private SimBoolean m_escEepromFault;
  private SimBoolean m_firmwareFault;
  private SimBoolean m_brownoutWarning;
  private SimBoolean m_overCurrentWarning;
  private SimBoolean m_escEepromWarning;
  private SimBoolean m_extEepromWarning;
  private SimBoolean m_sensorWarning;
  private SimBoolean m_stallWarning;
  private SimBoolean m_hasResetWarning;
  private SimBoolean m_otherWarning;
  private SimBoolean m_otherStickyFault;
  private SimBoolean m_motorTypeStickyFault;
  private SimBoolean m_sensorStickyFault;
  private SimBoolean m_canStickyFault;
  private SimBoolean m_temperatureStickyFault;
  private SimBoolean m_drvStickyFault;
  private SimBoolean m_escEepromStickyFault;
  private SimBoolean m_firmwareStickyFault;
  private SimBoolean m_brownoutStickyWarning;
  private SimBoolean m_overCurrentStickyWarning;
  private SimBoolean m_escEepromStickyWarning;
  private SimBoolean m_extEepromStickyWarning;
  private SimBoolean m_sensorStickyWarning;
  private SimBoolean m_stallStickyWarning;
  private SimBoolean m_hasResetStickyWarning;
  private SimBoolean m_otherStickyWarning;

  private SparkBase m_spark;
  private String simDeviceName;

  /**
   * Create a Fault Manager object, which allows you to get and set the status of simulated faults
   * on your simulated Spark MAX. Constructing this object will also cause the Fault Manager to
   * appear in the sim gui. The state of faults can be fetched on the original Spark Object.
   *
   * @param motor The CANSparkMax associated with the Fault Manager
   */
  public SparkSimFaultManager(SparkMax motor) {
    simDeviceName = ("SPARK MAX [" + motor.getDeviceId() + "] FAULT MANAGER");
    m_spark = motor;
    setupSimDevice();
  }

  /**
   * Create a Fault Manager object, which allows you to set the status of simulated faults on your
   * simulated Spark Flex. Constructing this object will also cause the Fault Manager to appear in
   * the sim gui. The state of faults can be fetched on the original Spark Object.
   *
   * @param motor The CANSparkMax associated with the Fault Manager
   */
  public SparkSimFaultManager(SparkFlex motor) {
    simDeviceName = ("SPARK Flex [" + motor.getDeviceId() + "] FAULT MANAGER");
    m_spark = motor;
    setupSimDevice();
  }

  // internal setup helper
  private boolean setupSimDevice() {
    m_spark.createSimFaultManager();
    SimDeviceSim faultManager = new SimDeviceSim(simDeviceName);
    m_otherFault = faultManager.getBoolean("Other Fault");
    m_motorTypeFault = faultManager.getBoolean("Motor Type Fault");
    m_sensorFault = faultManager.getBoolean("Sensor Fault");
    m_canFault = faultManager.getBoolean("CAN Fault");
    m_temperatureFault = faultManager.getBoolean("Temperature Fault");
    m_drvFault = faultManager.getBoolean("DRV Fault");
    m_escEepromFault = faultManager.getBoolean("ESC Eeprom Fault");
    m_firmwareFault = faultManager.getBoolean("Firmware Fault");
    m_brownoutWarning = faultManager.getBoolean("Brownout Warning");
    m_overCurrentWarning = faultManager.getBoolean("Over Current Warning");
    m_escEepromWarning = faultManager.getBoolean("ESC Eeprom Warning");
    m_extEepromWarning = faultManager.getBoolean("EXT Eeprom Warning");
    m_sensorWarning = faultManager.getBoolean("Sensor Warning");
    m_stallWarning = faultManager.getBoolean("Stall Warning");
    m_hasResetWarning = faultManager.getBoolean("Has Reset Warning");
    m_otherWarning = faultManager.getBoolean("Other Warning");
    m_otherStickyFault = faultManager.getBoolean("Other Sticky Fault");
    m_motorTypeStickyFault = faultManager.getBoolean("Motor Type Sticky Fault");
    m_sensorStickyFault = faultManager.getBoolean("Sensor Sticky Fault");
    m_canStickyFault = faultManager.getBoolean("CAN Sticky Fault");
    m_temperatureStickyFault = faultManager.getBoolean("Temperature Sticky Fault");
    m_drvStickyFault = faultManager.getBoolean("DRV Sticky Fault");
    m_escEepromStickyFault = faultManager.getBoolean("ESC Eeprom Sticky Fault");
    m_firmwareStickyFault = faultManager.getBoolean("Firmware Sticky Fault");
    m_brownoutStickyWarning = faultManager.getBoolean("Brownout Sticky Warning");
    m_overCurrentStickyWarning = faultManager.getBoolean("Over Current Sticky Warning");
    m_escEepromStickyWarning = faultManager.getBoolean("ESC Eeprom Sticky Warning");
    m_extEepromStickyWarning = faultManager.getBoolean("EXT Eeprom Sticky Warning");
    m_sensorStickyWarning = faultManager.getBoolean("Sensor Sticky Warning");
    m_stallStickyWarning = faultManager.getBoolean("Stall Sticky Warning");
    m_hasResetStickyWarning = faultManager.getBoolean("Has Reset Sticky Warning");
    m_otherStickyWarning = faultManager.getBoolean("Other Sticky Warning");

    return m_otherFault == null;
  }

  // internal setup helper
  private boolean checkAndSetupSimDevice() {
    if (m_otherFault == null) {
      return setupSimDevice();
    }
    return false;
  }

  /**
   * Set the state of the simulated faults of the device.
   *
   * <p>Use device.getFaults() to get the object and modify the parameters.
   *
   * @param faults a Faults object indicating the state of the faults
   */
  public void setFaults(SparkBase.Faults faults) {
    if (checkAndSetupSimDevice()) return;

    m_otherFault.set(faults.other);
    m_motorTypeFault.set(faults.motorType);
    m_sensorFault.set(faults.sensor);
    m_canFault.set(faults.can);
    m_temperatureFault.set(faults.temperature);
    m_drvFault.set(faults.gateDriver);
    m_escEepromFault.set(faults.escEeprom);
    m_firmwareFault.set(faults.firmware);
  }

  /**
   * Set the state of the simulated sticky faults of the device.
   *
   * <p>Use device.getStickyFaults() to get the object and modify the parameters.
   *
   * @param faults a Faults object indicating the state of the sticky faults
   */
  public void setStickyFaults(SparkBase.Faults faults) {
    if (checkAndSetupSimDevice()) return;

    m_otherStickyFault.set(faults.other);
    m_motorTypeStickyFault.set(faults.motorType);
    m_sensorStickyFault.set(faults.sensor);
    m_canStickyFault.set(faults.can);
    m_temperatureStickyFault.set(faults.temperature);
    m_drvStickyFault.set(faults.gateDriver);
    m_escEepromStickyFault.set(faults.escEeprom);
    m_firmwareStickyFault.set(faults.firmware);
  }

  /**
   * Set the state of the simulated warnings of the device.
   *
   * <p>Use device.getWarnings() to get the object and modify the parameters.
   *
   * @param warnings a Warnings object indicating the state of the warnings
   */
  public void setWarnings(SparkBase.Warnings warnings) {
    if (checkAndSetupSimDevice()) return;

    m_brownoutWarning.set(warnings.brownout);
    m_overCurrentWarning.set(warnings.overcurrent);
    m_escEepromWarning.set(warnings.escEeprom);
    m_extEepromWarning.set(warnings.extEeprom);
    m_sensorWarning.set(warnings.sensor);
    m_stallWarning.set(warnings.stall);
    m_hasResetWarning.set(warnings.hasReset);
    m_otherWarning.set(warnings.other);
  }

  /**
   * Set the state of the simulated sticky warnings of the device.
   *
   * <p>Use device.getStickyWarnings() to get the object and modify the parameters.
   *
   * @param warnings a Warnings object indicating the state of the sticky warnings
   */
  public void setStickyWarnings(SparkBase.Warnings warnings) {
    if (checkAndSetupSimDevice()) return;

    m_brownoutStickyWarning.set(warnings.brownout);
    m_overCurrentStickyWarning.set(warnings.overcurrent);
    m_escEepromStickyWarning.set(warnings.escEeprom);
    m_extEepromStickyWarning.set(warnings.extEeprom);
    m_sensorStickyWarning.set(warnings.sensor);
    m_stallStickyWarning.set(warnings.stall);
    m_hasResetStickyWarning.set(warnings.hasReset);
    m_otherStickyWarning.set(warnings.other);
  }
}
