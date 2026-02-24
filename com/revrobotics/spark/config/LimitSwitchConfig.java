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

import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig.DataPortConfig;

public class LimitSwitchConfig extends BaseConfig {
  public enum Behavior {
    kKeepMovingMotor(0),
    kStopMovingMotor(1),
    kKeepMovingMotorAndSetPosition(2),
    kStopMovingMotorAndSetPosition(3);

    @SuppressWarnings("MemberName")
    public final int value;

    Behavior(int value) {
      this.value = value;
    }

    public static Behavior fromId(int id) {
      switch (id) {
        case 0:
          return kKeepMovingMotor;
        case 1:
          return kStopMovingMotor;
        case 2:
          return kKeepMovingMotorAndSetPosition;
        case 3:
          return kStopMovingMotorAndSetPosition;
        default:
          return kStopMovingMotor;
      }
    }
  }

  public enum Type {
    kNormallyOpen(0),
    kNormallyClosed(1);

    @SuppressWarnings("MemberName")
    public final int value;

    Type(int value) {
      this.value = value;
    }

    public static Type fromId(int id) {
      switch (id) {
        case 1:
          return kNormallyClosed;
        default:
          return kNormallyOpen;
      }
    }
  }

  /** Create a new object to configure a LimitSwitch. */
  public LimitSwitchConfig() {
    super(CANType.kSpark);
  }

  /**
   * Applies settings from another {@link LimitSwitchConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link LimitSwitchConfig} to copy settings from
   * @return The updated {@link LimitSwitchConfig} for method chaining
   */
  public LimitSwitchConfig apply(LimitSwitchConfig config) {
    super.apply(config);
    return this;
  }

  /**
   * Configures the data port to use limit switches, which is specifically required for SPARK MAX.
   *
   * <p>NOTE: This method is only necessary when using limit switches with a SPARK MAX without
   * configuring any of its settings
   *
   * <p>IMPORTANT: SPARK MAX does not support using limit switches in conjunction with an alternate
   * encoder.
   *
   * @return The modified {@link LimitSwitchConfig} object for method chaining
   */
  public LimitSwitchConfig setSparkMaxDataPortConfig() {
    putParameter(
        SparkParameters.kCompatibilityPortConfig.value,
        DataPortConfig.kLimitSwitchesAndAbsoluteEncoder.value);
    return this;
  }

  /**
   * Set whether to enable/disable motor shutdown based on the forward limit switch state. This does
   * not not affect the result of the isPressed() command.
   *
   * @param enabled True for halting the motor when triggered
   * @return The modified {@link LimitSwitchConfig} object for method chaining
   * @deprecated Use the {@link forwardLimitSwitchTriggerBehavior(Behavior)} instead
   */
  @Deprecated
  public LimitSwitchConfig forwardLimitSwitchEnabled(boolean enabled) {
    return forwardLimitSwitchTriggerBehavior(
        enabled ? Behavior.kStopMovingMotor : Behavior.kKeepMovingMotor);
  }

  /**
   * Set the trigger behavior based on the forward limit switch state. This does not not affect the
   * result of the isPressed() command.
   *
   * @param behavior The trigger behavior
   * @return The modified {@link LimitSwitchConfig} object for method chaining
   */
  public LimitSwitchConfig forwardLimitSwitchTriggerBehavior(Behavior behavior) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kHardLimitFwdEn.value, behavior.value);
    return this;
  }

  /**
   * Set the normal state of the forward limit switch.
   *
   * @param type kNormallyOpen or kNormallyClosed
   * @return The modified {@link LimitSwitchConfig} object for method chaining
   */
  public LimitSwitchConfig forwardLimitSwitchType(Type type) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kLimitSwitchFwdPolarity.value, type == Type.kNormallyClosed);
    return this;
  }

  /**
   * Set the triggered position value of the forward limit switch (used when the enable mode is set
   * to kEnabled_SetValueOnTrigger).
   *
   * @param position user specified position value
   * @return The modified {@link LimitSwitchConfig} object for method chaining
   */
  public LimitSwitchConfig forwardLimitSwitchPosition(double position) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kLimitSwitchFwdPosition.value, (float) position);
    return this;
  }

  /**
   * Set whether to enable/disable motor shutdown based on the reverse limit switch state. This does
   * not not affect the result of the isPressed() command.
   *
   * @param enabled True for halting the motor when triggered
   * @return The modified {@link LimitSwitchConfig} object for method chaining
   * @deprecated Use the {@link reverseLimitSwitchTriggerBehavior(Behavior)} instead
   */
  @Deprecated
  public LimitSwitchConfig reverseLimitSwitchEnabled(boolean enabled) {
    return reverseLimitSwitchTriggerBehavior(
        enabled ? Behavior.kStopMovingMotor : Behavior.kKeepMovingMotor);
  }

  /**
   * Set the trigger behavior based on the reverse limit switch state. This does not not affect the
   * result of the isPressed() command.
   *
   * @param behavior The trigger behavior
   * @return The modified {@link LimitSwitchConfig} object for method chaining
   */
  public LimitSwitchConfig reverseLimitSwitchTriggerBehavior(Behavior behavior) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kHardLimitRevEn.value, behavior.value);
    return this;
  }

  /**
   * Set the normal state of the reverse limit switch.
   *
   * @param type kNormallyOpen or kNormallyClosed
   * @return The modified {@link LimitSwitchConfig} object for method chaining
   */
  public LimitSwitchConfig reverseLimitSwitchType(Type type) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kLimitSwitchRevPolarity.value, type == Type.kNormallyClosed);
    return this;
  }

  /**
   * Set the triggered position value of the reverse limit switch (used when the enable mode is set
   * to kEnabled_SetValueOnTrigger).
   *
   * @param position user specified position value
   * @return The modified {@link LimitSwitchConfig} object for method chaining
   */
  public LimitSwitchConfig reverseLimitSwitchPosition(double position) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kLimitSwitchRevPosition.value, (float) position);
    return this;
  }

  /**
   * Specifies the feedback sensor that the triggered position value is set on. This applies for
   * both forward and reverse limit switches.
   *
   * @param sensor The feedback sensor to set the position value on
   * @return The modified LimitSwitchConfig object for method chaining
   */
  public LimitSwitchConfig limitSwitchPositionSensor(FeedbackSensor sensor) {
    setSparkMaxDataPortConfig();

    putParameter(SparkParameters.kLimitSwitchPositionSensor.value, sensor.value);
    return this;
  }
}
