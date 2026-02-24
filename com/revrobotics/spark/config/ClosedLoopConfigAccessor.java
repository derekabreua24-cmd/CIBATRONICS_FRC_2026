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

package com.revrobotics.spark.config;

import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class ClosedLoopConfigAccessor {
  /**
   * Accessor for parameters relating to the MAXMotion. To configure these values, use {@link
   * MAXMotionConfig} and call {@link com.revrobotics.spark.SparkBase#configure(SparkBaseConfig,
   * com.revrobotics.ResetMode, com.revrobotics.PersistMode)}
   */
  public final MAXMotionConfigAccessor maxMotion;

  /**
   * Accessor for parameters relating to FeedForward. To configure these values, use {@link
   * FeedForwardConfig} and call {@link com.revrobotics.spark.SparkBase#configure(SparkBaseConfig,
   * com.revrobotics.ResetMode, com.revrobotics.PersistMode)}
   */
  public final FeedForwardConfigAccessor feedForward;

  private final long sparkHandle;

  protected ClosedLoopConfigAccessor(long sparkHandle) {
    this.sparkHandle = sparkHandle;

    maxMotion = new MAXMotionConfigAccessor(sparkHandle);
    feedForward = new FeedForwardConfigAccessor(sparkHandle);
  }

  public double getP() {
    return getP(ClosedLoopSlot.kSlot0);
  }

  public double getI() {
    return getI(ClosedLoopSlot.kSlot0);
  }

  public double getD() {
    return getD(ClosedLoopSlot.kSlot0);
  }

  /**
   * @deprecated Use {@link FeedForwardConfigAccessor#getkV()} to get this gain
   */
  @Deprecated(forRemoval = true)
  public double getFF() {
    return getFF(ClosedLoopSlot.kSlot0);
  }

  public double getDFilter() {
    return getDFilter(ClosedLoopSlot.kSlot0);
  }

  public double getIZone() {
    return getIZone(ClosedLoopSlot.kSlot0);
  }

  public double getMinOutput() {
    return getMinOutput(ClosedLoopSlot.kSlot0);
  }

  public double getMaxOutput() {
    return getMaxOutput(ClosedLoopSlot.kSlot0);
  }

  public double getP(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kP_0.value + slot.value * 8);
  }

  public double getI(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kI_0.value + slot.value * 8);
  }

  public double getD(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kD_0.value + slot.value * 8);
  }

  /**
   * @deprecated Use {@link FeedForwardConfigAccessor#getkV(ClosedLoopSlot slot)} to get this gain
   */
  @Deprecated(forRemoval = true)
  public double getFF(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kV_0.value + slot.value * 8);
  }

  public double getDFilter(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kDFilter_0.value + slot.value * 8);
  }

  public double getIZone(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kIZone_0.value + slot.value * 8);
  }

  public double getMinOutput(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kOutputMin_0.value + slot.value * 8);
  }

  public double getMaxOutput(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kOutputMax_0.value + slot.value * 8);
  }

  public double getMaxIAccumulation() {
    return getMaxIAccumulation(ClosedLoopSlot.kSlot0);
  }

  public double getMaxIAccumulation(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kIMaxAccum_0.value + slot.value * 4);
  }

  public double getAllowedClosedLoopError(ClosedLoopSlot slot) {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kAllowedClosedLoopError_0.value + slot.value * 4);
  }

  public boolean getPositionWrappingEnabled() {
    return CANSparkJNI.c_Spark_GetParameterBool(
        sparkHandle, SparkParameters.kPositionPIDWrapEnable.value);
  }

  public double getPositionWrappingMinInput() {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kPositionPIDMinInput.value);
  }

  public double getPositionWrappingMaxInput() {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kPositionPIDMaxInput.value);
  }

  public FeedbackSensor getFeedbackSensor() {
    int value =
        CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameters.kClosedLoopControlSensor.value);

    return FeedbackSensor.fromId(value);
  }
}
