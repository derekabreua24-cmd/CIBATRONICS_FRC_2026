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

package com.revrobotics.servohub;

import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.jni.CANServoHubJNI;
import com.revrobotics.jni.REVLibJNI;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoHubConfigAccessor;

public class ServoHub extends ServoHubLowLevel {
  /*
   * @deprecated Use {@link com.revrobotics.ResetMode} instead.
   */
  @Deprecated(since = "2026", forRemoval = true)
  public enum ResetMode {
    kNoResetSafeParameters(0),
    kResetSafeParameters(1);

    @SuppressWarnings("MemberName")
    public final int value;

    ResetMode(int value) {
      this.value = value;
    }
  }

  private ServoChannel[] servoChannels = new ServoChannel[ServoChannel.kNumServoChannels];

  /**
   * Accessor for ServoHub parameter values. This object contains fields and methods to retrieve
   * parameters that have been applied to the device. To set parameters, see {@link ServoHubConfig}
   * and {@link ServoHub#configure(ServoHubConfig, com.revrobotics.ResetMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final ServoHubConfigAccessor configAccessor;

  /**
   * Create a new object to control a ServoHub Servo Controller
   *
   * @param deviceId The device ID.
   */
  public ServoHub(int deviceId) {
    super(deviceId);
    configAccessor = new ServoHubConfigAccessor(servoHubHandle);

    for (int i = 0; i < servoChannels.length; ++i) {
      servoChannels[i] = new ServoChannel(ServoChannel.ChannelId.fromInt(i), this);
    }
  }

  /**
   * Set the configuration for the ServoHub.
   *
   * <p>If {@code resetMode} is {@link com.revrobotics.ResetMode#kResetSafeParameters}, this method
   * will reset safe writable parameters to their default values before setting the given
   * configuration.
   *
   * @param config The desired ServoHub configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError configure(ServoHubConfig config, com.revrobotics.ResetMode resetMode) {
    throwIfClosed();
    REVLibError status =
        REVLibError.fromInt(
            CANServoHubJNI.c_ServoHub_Configure(
                servoHubHandle,
                config.flatten(),
                resetMode == com.revrobotics.ResetMode.kResetSafeParameters));

    if (status != REVLibError.kOk) {
      throw new IllegalStateException(REVLibJNI.c_REVLib_ErrorFromCode(status.value));
    }

    return status;
  }

  /**
   * Set the configuration for the ServoHub.
   *
   * <p>If {@code resetMode} is {@link ResetMode#kResetSafeParameters}, this method will reset safe
   * writable parameters to their default values before setting the given configuration.
   *
   * @param config The desired ServoHub configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @return {@link REVLibError#kOk} if successful
   * @deprecated Use {@link ServoHub#configure(ServoHubConfig, com.revrobotics.ResetMode)} instead.
   */
  @Deprecated(since = "2026", forRemoval = true)
  public REVLibError configure(ServoHubConfig config, ResetMode resetMode) {
    throwIfClosed();
    REVLibError status =
        REVLibError.fromInt(
            CANServoHubJNI.c_ServoHub_Configure(
                servoHubHandle, config.flatten(), resetMode == ResetMode.kResetSafeParameters));

    if (status != REVLibError.kOk) {
      throw new IllegalStateException(REVLibJNI.c_REVLib_ErrorFromCode(status.value));
    }

    return status;
  }

  /**
   * Set the configuration for the ServoHub without waiting for a response.
   *
   * <p>If {@code resetMode} is {@link ResetMode#kResetSafeParameters}, this method will reset safe
   * writable parameters to their default values before setting the given configuration.
   *
   * <p>NOTE: This method will immediately return {@link REVLibError#kOk} and the action will be
   * done in the background. Any errors that occur will be reported to the driver station.
   *
   * @param config The desired ServoHub configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @return {@link REVLibError#kOk}
   * @see #configure(ServoHubConfig, ResetMode)
   */
  public REVLibError configureAsync(ServoHubConfig config, ResetMode resetMode) {
    throwIfClosed();
    return REVLibError.fromInt(
        CANServoHubJNI.c_ServoHub_ConfigureAsync(
            servoHubHandle, config.flatten(), resetMode == ResetMode.kResetSafeParameters));
  }

  /**
   * Get whether the ServoHub has one or more active faults.
   *
   * @return true if there is an active fault
   * @see #getFaults()
   */
  public boolean hasActiveFault() {
    throwIfClosed();
    return CANServoHubJNI.c_ServoHub_GetFaults(servoHubHandle) != 0;
  }

  /**
   * Get whether the ServoHub has one or more sticky faults.
   *
   * @return true if there is a sticky fault
   * @see #getStickyFaults()
   */
  public boolean hasStickyFault() {
    throwIfClosed();
    return CANServoHubJNI.c_ServoHub_GetStickyFaults(servoHubHandle) != 0;
  }

  /**
   * Get whether the ServoHub has one or more active warnings.
   *
   * @return true if there is an active warning
   * @see #getWarnings()
   */
  public boolean hasActiveWarning() {
    throwIfClosed();
    return CANServoHubJNI.c_ServoHub_GetWarnings(servoHubHandle) != 0;
  }

  /**
   * Get whether the ServoHub has one or more sticky warnings.
   *
   * @return true if there is a sticky warning
   * @see #getStickyWarnings()
   */
  public boolean hasStickyWarning() {
    throwIfClosed();
    return CANServoHubJNI.c_ServoHub_GetStickyWarnings(servoHubHandle) != 0;
  }

  public static class Faults {
    public final boolean regulatorPowerGood;
    public final boolean hardware;
    public final boolean firmware;
    public final boolean lowBattery;
    public final int rawBits;

    public Faults(int faults) {
      rawBits = faults;
      regulatorPowerGood = (faults & 0x1) != 0;
      hardware = (faults & 0x2) != 0;
      firmware = (faults & 0x4) != 0;
      lowBattery = (faults & 0x8) != 0;
    }
  }

  /**
   * Get the active faults that are currently present on the ServoHub. Faults are fatal errors that
   * prevent the motor from running.
   *
   * @return A struct with each fault and their active value
   */
  public Faults getFaults() {
    throwIfClosed();
    return new Faults(CANServoHubJNI.c_ServoHub_GetFaults(servoHubHandle));
  }

  /**
   * Get the sticky faults that were present on the ServoHub at one point since the sticky faults
   * were last cleared. Faults are fatal errors that prevent the motor from running.
   *
   * <p>Sticky faults can be cleared with {@link ServoHub#clearFaults()}.
   *
   * @return A struct with each fault and their sticky value
   */
  public Faults getStickyFaults() {
    throwIfClosed();
    return new Faults(CANServoHubJNI.c_ServoHub_GetStickyFaults(servoHubHandle));
  }

  public static class Warnings {
    public final boolean brownout;
    public final boolean canWarning;
    public final boolean canBusOff;
    public final boolean hasReset;
    public final boolean channel0Overcurrent;
    public final boolean channel1Overcurrent;
    public final boolean channel2Overcurrent;
    public final boolean channel3Overcurrent;
    public final boolean channel4Overcurrent;
    public final boolean channel5Overcurrent;
    public final int rawBits;

    public Warnings(int warnings) {
      rawBits = warnings;
      brownout = (warnings & 0x1) != 0;
      canWarning = (warnings & 0x2) != 0;
      canBusOff = (warnings & 0x4) != 0;
      hasReset = (warnings & 0x8) != 0;
      channel0Overcurrent = (warnings & 0x10) != 0;
      channel1Overcurrent = (warnings & 0x20) != 0;
      channel2Overcurrent = (warnings & 0x40) != 0;
      channel3Overcurrent = (warnings & 0x80) != 0;
      channel4Overcurrent = (warnings & 0x100) != 0;
      channel5Overcurrent = (warnings & 0x200) != 0;
    }
  }

  /**
   * Get the active warnings that are currently present on the ServoHub. Warnings are non-fatal
   * errors.
   *
   * @return A struct with each warning and their active value
   */
  public Warnings getWarnings() {
    throwIfClosed();
    return new Warnings(CANServoHubJNI.c_ServoHub_GetWarnings(servoHubHandle));
  }

  /**
   * Get the sticky warnings that were present on the ServoHub at one point since the sticky
   * warnings were last cleared. Warnings are non-fatal errors.
   *
   * <p>Sticky warnings can be cleared with {@link ServoHub#clearFaults()}.
   *
   * @return A struct with each warning and their sticky value
   */
  public Warnings getStickyWarnings() {
    throwIfClosed();
    return new Warnings(CANServoHubJNI.c_ServoHub_GetStickyWarnings(servoHubHandle));
  }

  /**
   * Clears all sticky faults.
   *
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError clearFaults() {
    throwIfClosed();
    return REVLibError.fromInt(CANServoHubJNI.c_ServoHub_ClearFaults(servoHubHandle));
  }

  /**
   * @return The voltage fed into the servo controller.
   */
  public double getDeviceVoltage() {
    throwIfClosed();
    return (double) CANServoHubJNI.c_ServoHub_GetDeviceVoltage(servoHubHandle);
  }

  /**
   * @return The servo controller's output current in Amps.
   */
  public double getDeviceCurrent() {
    throwIfClosed();
    return (double) CANServoHubJNI.c_ServoHub_GetDeviceCurrent(servoHubHandle);
  }

  /**
   * @return The voltage fed to the actual servos.
   */
  public double getServoVoltage() {
    throwIfClosed();
    return (double) CANServoHubJNI.c_ServoHub_GetServoVoltage(servoHubHandle);
  }

  /**
   * Returns an object to control a specific servo channel.
   *
   * @param channelId The specific servo channel to get
   * @return The specified ServoChannel
   */
  public ServoChannel getServoChannel(ServoChannel.ChannelId channelId) {
    throwIfClosed();
    return servoChannels[channelId.value];
  }

  public enum Bank {
    kBank0_2(0),
    kBank3_5(1);

    @SuppressWarnings("MemberName")
    public final int value;

    Bank(int value) {
      this.value = value;
    }
  }

  /**
   * Set the Pulse Period for servo channels 0-2 or servo channels 3-5.
   *
   * @param bank The bank of channels (0-2 or 3-5) to set
   * @param pulsePeriod_us The pulse period in microseconds
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setBankPulsePeriod(Bank bank, int pulsePeriod_us) {
    throwIfClosed();
    return REVLibError.fromInt(
        CANServoHubJNI.c_ServoHub_SetBankPulsePeriod(servoHubHandle, bank.value, pulsePeriod_us));
  }
}
