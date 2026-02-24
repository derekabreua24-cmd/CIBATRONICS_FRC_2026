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

import com.revrobotics.NativeResourceCleaner;
import com.revrobotics.REVDevice;
import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANServoHubJNI;
import java.util.concurrent.atomic.AtomicBoolean;
import org.jspecify.annotations.Nullable;

public abstract class ServoHubLowLevel extends NativeResourceCleaner
    implements REVDevice, AutoCloseable {
  protected final long servoHubHandle;
  private final AtomicBoolean isClosed = new AtomicBoolean(false);
  private final int deviceId;
  private String firmwareString = "";

  /**
   * Create a new object to control a ServoHub Servo Controller
   *
   * @param deviceId The device ID.
   */
  public ServoHubLowLevel(int deviceId) {
    this.deviceId = deviceId;

    if (CANServoHubJNI.c_ServoHub_RegisterId(deviceId) == REVLibError.kDuplicateCANId.value) {
      throw new IllegalStateException(
          "A CANServoHub instance has already been created with this device ID: " + deviceId);
    }
    servoHubHandle = CANServoHubJNI.c_ServoHub_Create(deviceId);
    registerCleaner(servoHubHandle);
  }

  /** Closes the ServoHub Controller */
  @Override
  public void close() {
    boolean wasClosed = isClosed.getAndSet(true);
    if (wasClosed) {
      return;
    }

    CANServoHubJNI.c_ServoHub_Close(servoHubHandle);
  }

  @Override
  protected OnClean getCleanAction() {
    return CANServoHubJNI::c_ServoHub_Destroy;
  }

  /**
   * Get the configured Device ID of the ServoHub.
   *
   * @return int device ID
   */
  public int getDeviceId() {
    throwIfClosed();
    return deviceId;
  }

  public class FirmwareVersion {
    private int firmwareFix;
    private int firmwareMinor;
    private int firmwareYear;
    private int hardwareMinor;
    private int hardwareMajor;

    public int getYear() {
      return firmwareYear;
    }

    public int getMinor() {
      return firmwareMinor;
    }

    public int getFix() {
      return firmwareFix;
    }

    public int hardwareMajor() {
      return hardwareMajor;
    }

    public int hardwareMinor() {
      return hardwareMinor;
    }
  }

  /**
   * Get the firmware version of the ServoHub.
   *
   * @return FirmwareVersion tbd
   */
  public FirmwareVersion getFirmwareVersion() {
    throwIfClosed();
    FirmwareVersion version = new FirmwareVersion();
    CANServoHubJNI.c_ServoHub_GetFirmwareVersion(servoHubHandle, version);
    return version;
  }

  /**
   * Get the firmware version of the ServoHub as a string.
   *
   * @return String Human readable firmware version string
   */
  public String getFirmwareVersionString() {
    throwIfClosed();
    if (firmwareString == "") {
      FirmwareVersion version = getFirmwareVersion();

      StringBuilder firmwareString = new StringBuilder();
      firmwareString
          .append("fw v")
          .append(version.firmwareYear)
          .append(".")
          .append(version.firmwareMinor)
          .append(".")
          .append(version.firmwareFix)
          .append(", hw v")
          .append(version.hardwareMajor)
          .append(".")
          .append(version.hardwareMinor);

      this.firmwareString = firmwareString.toString();
    }
    return this.firmwareString;
  }

  /**
   * Set the amount of time to wait for a periodic status frame before returning a timeout error.
   * This timeout will apply to all periodic status frames for the ServoHub servo controller.
   *
   * <p>To prevent invalid timeout errors, the minimum timeout for a given periodic status is 2.1
   * times its period. To use the minimum timeout for all status frames, set timeout_ms to 0.
   *
   * <p>The default timeout is 500ms.
   *
   * @param timeout_ms The timeout in milliseconds
   */
  public void setPeriodicFrameTimeout(int timeout_ms) {
    throwIfClosed();
    CANServoHubJNI.c_ServoHub_SetPeriodicFrameTimeout(servoHubHandle, timeout_ms);
  }

  /**
   * Sets the timeout duration for waiting for CAN responses from the device.
   *
   * @param timeout_ms The timeout in milliseconds.
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setCANTimeout(int timeout_ms) {
    throwIfClosed();
    return REVLibError.fromInt(CANServoHubJNI.c_ServoHub_SetCANTimeout(servoHubHandle, timeout_ms));
  }

  /**
   * Set the maximum number of times to retry an RTR CAN frame. This applies to calls such as
   * GetFirmwareVersion where a request is made to the ServoHub and a response is expected. Anytime
   * sending the request or receiving the response fails, it will retry the request a number of
   * times, no more than the value set by this method. If an attempt succeeds, it will immediately
   * return. The minimum number of retries is 0, where only a single attempt will be made and will
   * return regardless of success or failure.
   *
   * <p>The default maximum is 5 retries.
   *
   * @param numRetries The maximum number of retries
   */
  public void setCANMaxRetries(int numRetries) {
    throwIfClosed();
    CANServoHubJNI.c_ServoHub_SetCANMaxRetries(servoHubHandle, numRetries);
  }

  /**
   * Set the control frame send period for the native CAN Send thread.
   *
   * @param periodMs The send period in milliseconds between 1ms and 100ms or set to 0 to disable
   *     periodic sends.
   */
  public void setControlFramePeriodMs(int periodMs) {
    throwIfClosed();
    CANServoHubJNI.c_ServoHub_SetControlFramePeriod(servoHubHandle, periodMs);
  }

  /**
   * Set the control frame send period for the native CAN Send thread.
   *
   * @return int The send period in milliseconds.
   */
  public int getControlFramePeriodMs() {
    throwIfClosed();
    return CANServoHubJNI.c_ServoHub_GetControlFramePeriod(servoHubHandle);
  }

  public class PeriodicStatus0 {
    public double voltage;
    public double servoVoltage;
    public double deviceCurrent;
    public boolean primaryHeartbeatLock;
    public boolean systemEnabled;
    public int communicationMode; // 0: None, 1: CAN, 2: RS-485
    public boolean programmingEnabled;
    public boolean activelyProgramming;
  }

  @Nullable
  public PeriodicStatus0 getPeriodicStatus0() {
    throwIfClosed();
    PeriodicStatus0 status0 = new PeriodicStatus0();
    boolean success = CANServoHubJNI.c_ServoHub_GetPeriodStatus0(servoHubHandle, status0);
    if (!success) {
      return null;
    }
    return status0;
  }

  public class PeriodicStatus1 {
    public boolean regulatorPowerGoodFault;
    public boolean brownout;
    public boolean canWarning;
    public boolean canBusOff;
    public boolean hardwareFault;
    public boolean firmwareFault;
    public boolean hasReset;
    public boolean channel0Overcurrent;
    public boolean channel1Overcurrent;
    public boolean channel2Overcurrent;
    public boolean channel3Overcurrent;
    public boolean channel4Overcurrent;
    public boolean channel5Overcurrent;
    public boolean stickyRegulatorPowerGoodFault;
    public boolean stickyBrownout;
    public boolean stickyCanWarning;
    public boolean stickyCanBusOff;
    public boolean stickyHardwareFault;
    public boolean stickyFirmwareFault;
    public boolean stickyHasReset;
    public boolean stickyChannel0Overcurrent;
    public boolean stickyChannel1Overcurrent;
    public boolean stickyChannel2Overcurrent;
    public boolean stickyChannel3Overcurrent;
    public boolean stickyChannel4Overcurrent;
    public boolean stickyChannel5Overcurrent;
  }

  @Nullable
  public PeriodicStatus1 getPeriodicStatus1() {
    throwIfClosed();
    PeriodicStatus1 status1 = new PeriodicStatus1();
    boolean success = CANServoHubJNI.c_ServoHub_GetPeriodStatus1(servoHubHandle, status1);
    if (!success) {
      return null;
    }
    return status1;
  }

  public class PeriodicStatus2 {
    public short channel0PulseWidth;
    public short channel1PulseWidth;
    public short channel2PulseWidth;
    public boolean channel0Enabled;
    public boolean channel1Enabled;
    public boolean channel2Enabled;
    public boolean channel0OutOfRange;
    public boolean channel1OutOfRange;
    public boolean channel2OutOfRange;
  }

  @Nullable
  public PeriodicStatus2 getPeriodicStatus2() {
    throwIfClosed();
    PeriodicStatus2 status2 = new PeriodicStatus2();
    boolean success = CANServoHubJNI.c_ServoHub_GetPeriodStatus2(servoHubHandle, status2);
    if (!success) {
      return null;
    }
    return status2;
  }

  public class PeriodicStatus3 {
    public short channel3PulseWidth;
    public short channel4PulseWidth;
    public short channel5PulseWidth;
    public boolean channel3Enabled;
    public boolean channel4Enabled;
    public boolean channel5Enabled;
    public boolean channel3OutOfRange;
    public boolean channel4OutOfRange;
    public boolean channel5OutOfRange;
  }

  @Nullable
  public PeriodicStatus3 getPeriodicStatus3() {
    throwIfClosed();
    PeriodicStatus3 status3 = new PeriodicStatus3();
    boolean success = CANServoHubJNI.c_ServoHub_GetPeriodStatus3(servoHubHandle, status3);
    if (!success) {
      return null;
    }
    return status3;
  }

  public class PeriodicStatus4 {
    public double channel0Current;
    public double channel1Current;
    public double channel2Current;
    public double channel3Current;
    public double channel4Current;
    public double channel5Current;
  }

  @Nullable
  public PeriodicStatus4 getPeriodicStatus4() {
    throwIfClosed();
    PeriodicStatus4 status4 = new PeriodicStatus4();
    boolean success = CANServoHubJNI.c_ServoHub_GetPeriodStatus4(servoHubHandle, status4);
    if (!success) {
      return null;
    }
    return status4;
  }

  /** Create the sim gui Fault Manager for this Servo Hub */
  public void createSimFaultManager() {
    CANServoHubJNI.c_ServoHub_CreateSimFaultManager(servoHubHandle);
  }

  protected void throwIfClosed() {
    if (isClosed.get()) {
      throw new IllegalStateException("This ServoHub object has previously been closed.");
    }
  }
}
