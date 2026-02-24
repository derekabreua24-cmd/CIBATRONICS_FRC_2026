/*
 * Copyright (c) 2025-2026 REV Robotics
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

import com.revrobotics.encoder.DetachedEncoder;

public abstract class DetachedEncoderJNI extends RevJNIWrapper {
  public static native long create(int id, int model);

  public static native int registerId(int id);

  public static native void clearFaults(long handle);

  public static native int getEncoderModel(long handle);

  public static native double getEncoderPosition(long handle);

  public static native double getEncoderVelocity(long handle);

  public static native double getEncoderAngle(long handle);

  public static native double getEncoderRawAngle(long handle);

  public static native int setEncoderPosition(long handle, double position);

  public static native int getFaultsBitfield(long handle);

  public static native int getStickyFaultsBitfield(long handle);

  public static native DetachedEncoder.FirmwareVersion getFirmwareVersion(long handle);

  public static native int getParameterType(int index);

  public static native int configure(long handle, String flattened, boolean reset);

  public static native void close(long handle);

  public static native void destroy(long handle);

  public static native boolean isInverted(long handle);

  public static native int getAverageDepth(long handle);

  public static native float getPositionConversionFactor(long handle);

  public static native float getVelocityConversionFactor(long handle);

  public static native float getAngleConversionFactor(long handle);

  public static native boolean isDutyCycleZeroCentered(long handle);

  public static native int getDutyCycleAverageDepth(long handle);

  public static native float getDutyCycleOffset(long handle);

  public static native float getDutyCycleStartPulseUs(long handle);

  public static native float getDutyCycleEndPulseUs(long handle);

  public static native float getDutyCyclePeriodUs(long handle);

  public static native DetachedEncoder.PeriodicStatus0 getStatus0(long handle);

  public static native DetachedEncoder.PeriodicStatus1 getStatus1(long handle);

  public static native DetachedEncoder.PeriodicStatus2 getStatus2(long handle);

  public static native DetachedEncoder.PeriodicStatus3 getStatus3(long handle);

  public static native DetachedEncoder.PeriodicStatus4 getStatus4(long handle);

  public static native int getEncoderVelocityPeriodMs(long handle);

  public static native int getEncoderPositionPeriodMs(long handle);

  public static native int getEncoderAnglePeriodMs(long handle);
}
