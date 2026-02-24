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

import com.revrobotics.spark.SparkLowLevel;
import org.jspecify.annotations.Nullable;

public class CANSparkJNI extends RevJNIWrapper {
  // CANSparkLowLevel
  public static native int c_Spark_RegisterId(int deviceId);

  public static native long c_Spark_Create(int deviceId, int motortype, int sparkModel);

  public static native void c_Spark_Close(long handle);

  public static native void c_Spark_Destroy(long handle);

  public static native int c_Spark_GetFirmwareVersion(long handle);

  // public static native int c_Spark_GetSerialNumber(long handle, int*
  // serialNumber[3]);
  public static native int c_Spark_GetDeviceId(long handle);

  public static native void c_Spark_SetPeriodicFrameTimeout(long handle, int timeoutMs);

  public static native void c_Spark_SetControlFramePeriod(long handle, int periodMs);

  public static native int c_Spark_GetControlFramePeriod(long handle);

  public static native int c_Spark_SetEncoderPosition(long handle, float position);

  public static native int c_Spark_ResetSafeParameters(long handle, boolean persist);

  public static native float c_Spark_SafeFloat(float f);

  public static native int c_Spark_SetpointCommand(
      long handle, float value, int ctrlType, int pidSlot, float arbFeedforward, int arbFFUnits);

  // CANSparkMax
  public static native int c_Spark_SetInverted(long handle, boolean inverted);

  public static native boolean c_Spark_GetInverted(long handle);

  public static native boolean c_Spark_IsFollower(long handle);

  public static native int c_Spark_GetFaults(long handle);

  public static native int c_Spark_GetStickyFaults(long handle);

  public static native int c_Spark_GetWarnings(long handle);

  public static native int c_Spark_GetStickyWarnings(long handle);

  public static native float c_Spark_GetBusVoltage(long handle);

  public static native float c_Spark_GetAppliedOutput(long handle);

  public static native void c_Spark_SetSimAppliedOutput(long handle, float value);

  public static native float c_Spark_GetOutputCurrent(long handle);

  public static native float c_Spark_GetMotorTemperature(long handle);

  public static native int c_Spark_ClearFaults(long handle);

  public static native int c_Spark_PersistParameters(long handle);

  public static native int c_Spark_SetCANTimeout(long handle, int timeoutMs);

  public static native void c_Spark_SetCANMaxRetries(long handle, int numRetries);

  // Digital Input
  public static native boolean c_Spark_GetLimitSwitch(long handle, int sw);

  public static native boolean c_Spark_GetSoftLimit(long handle, int sw);

  // Analog
  public static native float c_Spark_GetAnalogPosition(long handle);

  public static native float c_Spark_GetAnalogVelocity(long handle);

  public static native float c_Spark_GetAnalogVoltage(long handle);

  // Relative Encoder
  public static native float c_Spark_GetEncoderPosition(long handle);

  public static native float c_Spark_GetEncoderVelocity(long handle);

  // Alternate Encoder
  public static native int c_Spark_SetAltEncoderPosition(long handle, float position);

  public static native float c_Spark_GetAltEncoderPosition(long handle);

  public static native float c_Spark_GetAltEncoderVelocity(long handle);

  public static native int c_Spark_GetDataPortConfig(long handle);

  public static native boolean c_Spark_IsDataPortConfigured(long handle);

  // Duty Cycle
  public static native float c_Spark_GetDutyCyclePosition(long handle);

  public static native float c_Spark_GetDutyCycleVelocity(long handle);

  // Closed Loop Controller
  public static native int c_Spark_SetIAccum(long handle, float iAccum);

  public static native float c_Spark_GetIAccum(long handle);

  public static native float c_Spark_GetSetpoint(long handle);

  public static native boolean c_Spark_IsAtSetpoint(long handle);

  public static native int c_Spark_GetSelectedSlot(long handle);

  public static native float c_Spark_GetMaxMotionSetpointPosition(long handle);

  public static native float c_Spark_GetMaxMotionSetpointVelocity(long handle);

  public static native int c_Spark_GetAPIMajorRevision();

  public static native int c_Spark_GetAPIMinorRevision();

  public static native int c_Spark_GetAPIBuildRevision();

  public static native int c_Spark_GetAPIVersion();

  public static native int c_Spark_GetLastError(long handle);

  public static native int c_Spark_GetSparkModel(long sparkHandle);

  public static native int c_Spark_SetParameterFloat32(long handle, int paramId, float value);

  public static native int c_Spark_SetParameterInt32(long handle, int paramId, int value);

  public static native int c_Spark_SetParameterUint32(long handle, int paramId, int value);

  public static native int c_Spark_SetParameterBool(long handle, int paramId, boolean value);

  public static native float c_Spark_GetParameterFloat32(long handle, int paramId);

  public static native int c_Spark_GetParameterInt32(long handle, int paramId);

  public static native int c_Spark_GetParameterUint32(long handle, int paramId);

  public static native boolean c_Spark_GetParameterBool(long handle, int paramId);

  public static native int c_Spark_GetParameterType(int paramId);

  public static native int c_Spark_GetMotorInterface(long handle);

  public static native int c_Spark_Configure(
      long handle, String flattenedString, boolean resetSafeParameters, boolean persistParameters);

  public static native int c_Spark_StartFollowerMode(long handle);

  public static native int c_Spark_StopFollowerMode(long handle);

  public static native float c_Spark_GetSimClosedLoopOutput(
      long handle, float setpoint, float pv, float dt);

  public static native float c_Spark_GetSimMAXMotionPositionControlOutput(long handle, float dt);

  public static native float c_Spark_GetSimMAXMotionVelocityControlOutput(long handle, float dt);

  public static native float c_Spark_GetSimCurrentLimitOutput(
      long handle, float appliedOutput, float current);

  public static native void c_Spark_CreateAbsoluteEncoderSim(long handle);

  public static native void c_Spark_CreateAlternateEncoderSim(long handle);

  public static native void c_Spark_CreateAnalogSensorSim(long handle);

  public static native void c_Spark_CreateForwardLimitSwitchSim(long handle);

  public static native void c_Spark_CreateReverseLimitSwitchSim(long handle);

  public static native void c_Spark_CreateSimFaultManager(long handle);

  public static native void c_Spark_CreateRelativeEncoderSim(long handle);

  public static native int c_Spark_ConfigureAsync(
      long handle, String flattenedString, boolean resetSafeParameters, boolean persistParameters);

  public static native int c_Spark_StartFollowerModeAsync(long handle);

  public static native int c_Spark_StopFollowerModeAsync(long handle);

  public static native SparkLowLevel.@Nullable PeriodicStatus0 c_Spark_GetPeriodicStatus0(
      long handle);

  public static native SparkLowLevel.@Nullable PeriodicStatus1 c_Spark_GetPeriodicStatus1(
      long handle);

  public static native SparkLowLevel.@Nullable PeriodicStatus2 c_Spark_GetPeriodicStatus2(
      long handle);

  public static native SparkLowLevel.@Nullable PeriodicStatus3 c_Spark_GetPeriodicStatus3(
      long handle);

  public static native SparkLowLevel.@Nullable PeriodicStatus4 c_Spark_GetPeriodicStatus4(
      long handle);

  public static native SparkLowLevel.@Nullable PeriodicStatus5 c_Spark_GetPeriodicStatus5(
      long handle);

  public static native SparkLowLevel.@Nullable PeriodicStatus6 c_Spark_GetPeriodicStatus6(
      long handle);

  public static native SparkLowLevel.@Nullable PeriodicStatus7 c_Spark_GetPeriodicStatus7(
      long handle);

  public static native SparkLowLevel.@Nullable PeriodicStatus8 c_Spark_GetPeriodicStatus8(
      long handle);

  public static native SparkLowLevel.@Nullable PeriodicStatus9 c_Spark_GetPeriodicStatus9(
      long handle);
}
