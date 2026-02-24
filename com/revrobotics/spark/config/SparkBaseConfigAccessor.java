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

public class SparkBaseConfigAccessor {
  private final long sparkHandle;

  /**
   * Accessor for parameters relating to the absolute encoder. To configure these values, use {@link
   * AbsoluteEncoderConfig} and call {@link
   * com.revrobotics.spark.SparkBase#configure(SparkBaseConfig, com.revrobotics.ResetMode,
   * com.revrobotics.PersistMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final AbsoluteEncoderConfigAccessor absoluteEncoder;

  /**
   * Accessor for parameters relating to the analog sensor. To configure these values, use {@link
   * AnalogSensorConfig} and call {@link com.revrobotics.spark.SparkBase#configure(SparkBaseConfig,
   * com.revrobotics.ResetMode, com.revrobotics.PersistMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final AnalogSensorConfigAccessor analogSensor;

  /**
   * Accessor for parameters relating to the primary encoder. To configure these values, use {@link
   * EncoderConfig} and call {@link com.revrobotics.spark.SparkBase#configure(SparkBaseConfig,
   * com.revrobotics.ResetMode, com.revrobotics.PersistMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final EncoderConfigAccessor encoder;

  /**
   * Accessor for parameters relating to the hardware limit switches. To configure these values, use
   * {@link LimitSwitchConfig} and call {@link
   * com.revrobotics.spark.SparkBase#configure(SparkBaseConfig, com.revrobotics.ResetMode,
   * com.revrobotics.PersistMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final LimitSwitchConfigAccessor limitSwitch;

  /**
   * Accessor for parameters relating to the closed loop controller. To configure these values, use
   * {@link ClosedLoopConfig} and call {@link
   * com.revrobotics.spark.SparkBase#configure(SparkBaseConfig, com.revrobotics.ResetMode,
   * com.revrobotics.PersistMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final ClosedLoopConfigAccessor closedLoop;

  /**
   * Accessor for parameters relating to the Software Limits. To configure these values, use {@link
   * SoftLimitConfig} and call {@link com.revrobotics.spark.SparkBase#configure(SparkBaseConfig,
   * com.revrobotics.ResetMode, com.revrobotics.PersistMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final SoftLimitConfigAccessor softLimit;

  /**
   * Accessor for parameters relating to the Status Signals. To configure these values, use {@link
   * SignalsConfig} and call {@link com.revrobotics.spark.SparkBase#configure(SparkBaseConfig,
   * com.revrobotics.ResetMode, com.revrobotics.PersistMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final SignalsConfigAccessor signals;

  protected SparkBaseConfigAccessor(long sparkHandle) {
    this.sparkHandle = sparkHandle;

    absoluteEncoder = new AbsoluteEncoderConfigAccessor(sparkHandle);
    analogSensor = new AnalogSensorConfigAccessor(sparkHandle);
    encoder = new EncoderConfigAccessor(sparkHandle);
    limitSwitch = new LimitSwitchConfigAccessor(sparkHandle);
    closedLoop = new ClosedLoopConfigAccessor(sparkHandle);
    softLimit = new SoftLimitConfigAccessor(sparkHandle);
    signals = new SignalsConfigAccessor(sparkHandle);
  }

  public SparkBaseConfig.IdleMode getIdleMode() {
    int value =
        CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameters.kIdleMode.value);

    return SparkBaseConfig.IdleMode.fromId(value);
  }

  public boolean getInverted() {
    return CANSparkJNI.c_Spark_GetParameterBool(sparkHandle, SparkParameters.kInverted.value);
  }

  public int getSmartCurrentLimit() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
        sparkHandle, SparkParameters.kSmartCurrentStallLimit.value);
  }

  public int getSmartCurrentFreeLimit() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
        sparkHandle, SparkParameters.kSmartCurrentFreeLimit.value);
  }

  public int getSmartCurrentRPMLimit() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
        sparkHandle, SparkParameters.kSmartCurrentConfig.value);
  }

  public double getSecondaryCurrentLimit() {
    return CANSparkJNI.c_Spark_GetParameterFloat32(sparkHandle, SparkParameters.kCurrentChop.value);
  }

  public int getSecondaryCurrentLimitChopCycles() {
    return CANSparkJNI.c_Spark_GetParameterInt32(
        sparkHandle, SparkParameters.kCurrentChopCycles.value);
  }

  public double getAdvanceCommutation() {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kCommutationAdvance.value);
  }

  public double getOpenLoopRampRate() {
    double value =
        CANSparkJNI.c_Spark_GetParameterFloat32(
            sparkHandle, SparkParameters.kOpenLoopRampRate.value);

    if (value == 0.0) {
      return value;
    }

    return 1.0 / value;
  }

  public double getClosedLoopRampRate() {
    double value =
        CANSparkJNI.c_Spark_GetParameterFloat32(
            sparkHandle, SparkParameters.kClosedLoopRampRate.value);

    if (value == 0.0) {
      return value;
    }

    return 1.0 / value;
  }

  public double getVoltageCompensation() {
    return CANSparkJNI.c_Spark_GetParameterFloat32(
        sparkHandle, SparkParameters.kCompensatedNominalVoltage.value);
  }

  public boolean getVoltageCompensationEnabled() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameters.kVoltageCompensationMode.value)
        != 0;
  }

  public int getFollowerModeLeaderId() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
        sparkHandle, SparkParameters.kFollowerModeLeaderId.value);
  }

  public boolean getFollowerModeInverted() {
    return CANSparkJNI.c_Spark_GetParameterBool(
        sparkHandle, SparkParameters.kFollowerModeIsInverted.value);
  }
}
