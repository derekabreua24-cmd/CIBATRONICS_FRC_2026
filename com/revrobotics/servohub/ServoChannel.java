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
import com.revrobotics.jni.CANServoHubJNI;

public class ServoChannel {
  public enum ChannelId {
    kChannelId0(0),
    kChannelId1(1),
    kChannelId2(2),
    kChannelId3(3),
    kChannelId4(4),
    kChannelId5(5);

    @SuppressWarnings("MemberName")
    public final int value;

    ChannelId(int value) {
      this.value = value;
    }

    public static ChannelId fromInt(int value) {
      switch (value) {
        case 0:
          return kChannelId0;
        case 1:
          return kChannelId1;
        case 2:
          return kChannelId2;
        case 3:
          return kChannelId3;
        case 4:
          return kChannelId4;
        case 5:
          return kChannelId5;
        default:
          return kChannelId0;
      }
    }
  }

  public static final int kNumServoChannels = 6;

  private ChannelId channelId;
  private final ServoHub servoHub;

  // package-private to the revrobotics package
  ServoChannel(ChannelId channelId, ServoHub device) {
    this.channelId = channelId;
    this.servoHub = device;
  }

  public ChannelId getChannelId() {
    servoHub.throwIfClosed();
    return channelId;
  }

  /**
   * @return The pulse width applied to this channel in microseconds.
   */
  public int getPulseWidth() {
    servoHub.throwIfClosed();
    return (int)
        CANServoHubJNI.c_ServoHub_GetChannelPulseWidth(servoHub.servoHubHandle, channelId.value);
  }

  /**
   * @return true if the channel is enabled; false, otherwise
   */
  public boolean isEnabled() {
    servoHub.throwIfClosed();
    return CANServoHubJNI.c_ServoHub_GetChannelEnabled(servoHub.servoHubHandle, channelId.value);
  }

  /**
   * @return The channel's output current in Amps.
   */
  public double getCurrent() {
    servoHub.throwIfClosed();
    return (double)
        CANServoHubJNI.c_ServoHub_GetChannelCurrent(servoHub.servoHubHandle, channelId.value);
  }

  /**
   * Sets the servo to the desired location based on the pulse width (in microseconds)
   *
   * @param pulseWidth_us The desired pulse width in microseconds
   */
  public REVLibError setPulseWidth(int pulseWidth_us) {
    servoHub.throwIfClosed();
    return REVLibError.fromInt(
        CANServoHubJNI.c_ServoHub_SetChannelPulseWidth(
            servoHub.servoHubHandle, channelId.value, pulseWidth_us));
  }

  /**
   * Enables/Disables the servo
   *
   * @param enabled true = enabled, false = disabled
   */
  public REVLibError setEnabled(boolean enabled) {
    servoHub.throwIfClosed();
    return REVLibError.fromInt(
        CANServoHubJNI.c_ServoHub_SetChannelEnabled(
            servoHub.servoHubHandle, channelId.value, enabled));
  }

  /**
   * Turns on/off the power to the servo
   *
   * @param powered true = powered on, false = powered off
   */
  public REVLibError setPowered(boolean powered) {
    servoHub.throwIfClosed();
    return REVLibError.fromInt(
        CANServoHubJNI.c_ServoHub_SetChannelPowered(
            servoHub.servoHubHandle, channelId.value, powered));
  }
}
