/*
 * Copyright (c) 2020-2025 REV Robotics
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

package com.revrobotics.jni;

import com.revrobotics.servohub.ServoHubLowLevel.FirmwareVersion;
import com.revrobotics.servohub.ServoHubLowLevel.PeriodicStatus0;
import com.revrobotics.servohub.ServoHubLowLevel.PeriodicStatus1;
import com.revrobotics.servohub.ServoHubLowLevel.PeriodicStatus2;
import com.revrobotics.servohub.ServoHubLowLevel.PeriodicStatus3;
import com.revrobotics.servohub.ServoHubLowLevel.PeriodicStatus4;
import com.revrobotics.servohub.config.ServoChannelConfig.PulseRange;

public class CANServoHubJNI extends RevJNIWrapper {
  // CANServoHubLowLevel
  public static native int c_ServoHub_RegisterId(int deviceId);

  public static native long c_ServoHub_Create(int deviceId);

  public static native void c_ServoHub_Close(long handle);

  public static native void c_ServoHub_Destroy(long handle);

  public static native void c_ServoHub_GetFirmwareVersion(long handle, FirmwareVersion version);

  public static native void c_ServoHub_SetPeriodicFrameTimeout(long handle, int timeout_ms);

  public static native int c_ServoHub_SetCANTimeout(long handle, int timeout_ms);

  public static native void c_ServoHub_SetCANMaxRetries(long handle, int numRetries);

  public static native void c_ServoHub_SetControlFramePeriod(long handle, int period_ms);

  public static native int c_ServoHub_GetControlFramePeriod(long handle);

  public static native boolean c_ServoHub_GetPeriodStatus0(long handle, PeriodicStatus0 status0);

  public static native boolean c_ServoHub_GetPeriodStatus1(long handle, PeriodicStatus1 status1);

  public static native boolean c_ServoHub_GetPeriodStatus2(long handle, PeriodicStatus2 status2);

  public static native boolean c_ServoHub_GetPeriodStatus3(long handle, PeriodicStatus3 status3);

  public static native boolean c_ServoHub_GetPeriodStatus4(long handle, PeriodicStatus4 status4);

  // CANServoHub
  public static native int c_ServoHub_Configure(
      long handle, String flattenedString, boolean resetSafeParameters);

  public static native int c_ServoHub_GetFaults(long handle);

  public static native int c_ServoHub_GetStickyFaults(long handle);

  public static native int c_ServoHub_GetWarnings(long handle);

  public static native int c_ServoHub_GetStickyWarnings(long handle);

  public static native int c_ServoHub_ClearFaults(long handle);

  public static native float c_ServoHub_GetDeviceVoltage(long handle);

  public static native float c_ServoHub_GetDeviceCurrent(long handle);

  public static native float c_ServoHub_GetServoVoltage(long handle);

  public static native int c_ServoHub_SetBankPulsePeriod(long handle, int bank, int pulsePeriod_us);

  // ServoChannel
  public static native int c_ServoHub_GetChannelPulseWidth(long handle, int channel);

  public static native boolean c_ServoHub_GetChannelEnabled(long handle, int channel);

  public static native float c_ServoHub_GetChannelCurrent(long handle, int channel);

  public static native void c_ServoHub_GetChannelPulseRange(
      long handle, int channel, PulseRange pulseRange);

  public static native boolean c_ServoHub_GetChannelDisableBehavior(long handle, int channel);

  public static native int c_ServoHub_SetChannelPulseWidth(
      long handle, int channel, int pulseWidth_us);

  public static native int c_ServoHub_SetChannelEnabled(long handle, int channel, boolean enabled);

  public static native int c_ServoHub_SetChannelPowered(long handle, int channel, boolean powered);

  // Async functions
  public static native int c_ServoHub_ConfigureAsync(
      long handle, String flattenedString, boolean resetSafeParameters);

  // CANServoHubDriver
  public static native int c_ServoHub_GetParameterType(int paramId);

  // Simulation
  public static native void c_ServoHub_CreateSimFaultManager(long handle);
}
