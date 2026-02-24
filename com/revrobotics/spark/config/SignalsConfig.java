/*
 * Copyright (c) 2024-2026 REV Robotics
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

public class SignalsConfig extends BaseConfig {
  /** Create a new object to configure Signals. */
  public SignalsConfig() {
    super(CANType.kSpark);
  }

  private void setPeriodMsCore(int parameterId, int periodMs) {
    Object value = getParameter(parameterId);

    if (value == null) {
      putParameter(parameterId, periodMs);
    } else {
      int currentPeriodMs = (int) value;
      putParameter(parameterId, Math.min(currentPeriodMs, periodMs));
    }
  }

  private void setAlwaysOnCore(int parameterId, boolean enabled) {
    // Ignore status 0
    if (parameterId == SparkParameters.kForceEnableStatus_0.value) {
      return;
    }

    Object value = getParameter(parameterId);

    if (value == null) {
      putParameter(parameterId, enabled);
    } else {
      boolean currentValue = (boolean) value;
      putParameter(parameterId, currentValue || enabled);
    }
  }

  /**
   * Applies settings from another {@link SignalsConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link SignalsConfig} to copy settings from
   * @return The updated {@link SignalsConfig} for method chaining
   */
  public SignalsConfig apply(SignalsConfig config) {
    super.apply(config);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getAppliedOutput() CANSparkBase.getAppliedOutput()}. The
   * default period is 10ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * <p><b>NOTE:</b> Applied output is used by other SPARK devices in follower mode. Setting too
   * long of a period should be avoided if this SPARK device is the leader, as it can degrade
   * follower mode performance.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig appliedOutputPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus0Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getAppliedOutput() CANSparkBase.getAppliedOutput()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   * @deprecated Calling this method will have no effect, as status 0 cannot be disabled.
   */
  @Deprecated
  public SignalsConfig appliedOutputAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_0.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getBusVoltage() CANSparkBase.getBusVoltage()}. The default
   * period is 10ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * <p><b>NOTE:</b> This signal shares a status frame with applied output which is used by other
   * SPARK devices in follower mode. Setting too long of a period should be avoided if this SPARK
   * device is the leader, as it can degrade follower mode performance.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig busVoltagePeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus0Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getBusVoltage() CANSparkBase.getBusVoltage()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   * @deprecated Calling this method will have no effect, as status 0 cannot be disabled.
   */
  @Deprecated
  public SignalsConfig busVoltageAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_0.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getOutputCurrent() CANSparkBase.getOutputCurrent()}. The
   * default period is 10ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * <p><b>NOTE:</b> This signal shares a status frame with applied output which is used by other
   * SPARK devices in follower mode. Setting too long of a period should be avoided if this SPARK
   * device is the leader, as it can degrade follower mode performance.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig outputCurrentPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus0Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getOutputCurrent() CANSparkBase.getOutputCurrent()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   * @deprecated Calling this method will have no effect, as status 0 cannot be disabled.
   */
  @Deprecated
  public SignalsConfig outputCurrentAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_0.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getMotorTemperature() CANSparkBase.getMotorTemperature()}. The
   * default period is 10ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * <p><b>NOTE:</b> This signal shares a status frame with applied output which is used by other
   * SPARK devices in follower mode. Setting too long of a period should be avoided if this SPARK
   * device is the leader, as it can degrade follower mode performance.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig motorTemperaturePeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus0Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getMotorTemperature() CANSparkBase.getMotorTemperature()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   * @deprecated Calling this method will have no effect, as status 0 cannot be disabled.
   */
  @Deprecated
  public SignalsConfig motorTemperatureAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_0.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkLimitSwitch#isPressed() SparkLimitSwitch.isPressed()}. The default
   * period is 10ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * <p><b>NOTE:</b> This signal shares a status frame with applied output which is used by other
   * SPARK devices in follower mode. Setting too long of a period should be avoided if this SPARK
   * device is the leader, as it can degrade follower mode performance.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig limitsPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus0Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkLimitSwitch#isPressed() SparkLimitSwitch.isPressed()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   * @deprecated Calling this method will have no effect, as status 0 cannot be disabled.
   */
  @Deprecated
  public SignalsConfig limitsAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_0.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getFaults() CANSparkBase.getFaults()} and {@link
   * com.revrobotics.spark.SparkBase#getStickyFaults() CANSparkBase.getStickyFaults()}. The default
   * period is 250ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig faultsPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus1Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getFaults() CANSparkBase.getFaults()} and {@link
   * com.revrobotics.spark.SparkBase#getStickyFaults() CANSparkBase.getStickyFaults()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig faultsAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_1.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getWarnings() CANSparkBase.getWarnings()} and {@link
   * com.revrobotics.spark.SparkBase#getStickyWarnings() CANSparkBase.getStickyWarnings()}. The
   * default period is 250ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig warningsPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus1Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkBase#getWarnings() CANSparkBase.getWarnings()} and {@link
   * com.revrobotics.spark.SparkBase#getStickyWarnings() CANSparkBase.getStickyWarnings()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig warningsAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_1.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkRelativeEncoder#getVelocity() SparkRelativeEncoder.getVelocity()}.
   * The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig primaryEncoderVelocityPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus2Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkRelativeEncoder#getVelocity() SparkRelativeEncoder.getVelocity()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig primaryEncoderVelocityAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_2.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkRelativeEncoder#getPosition() SparkRelativeEncoder.getPosition()}.
   * The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig primaryEncoderPositionPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus2Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkRelativeEncoder#getPosition() SparkRelativeEncoder.getPosition()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig primaryEncoderPositionAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_2.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkAnalogSensor#getVoltage() SparkAnalogSensor.getVoltage()}. The
   * default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig analogVoltagePeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus3Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkAnalogSensor#getVoltage() SparkAnalogSensor.getVoltage()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig analogVoltageAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_3.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkAnalogSensor#getVelocity() SparkAnalogSensor.getVelocity()}. The
   * default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig analogVelocityPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus3Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkAnalogSensor#getVelocity() SparkAnalogSensor.getVelocity()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig analogVelocityAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_3.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkAnalogSensor#getPosition() SparkAnalogSensor.getPosition()}. The
   * default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig analogPositionPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus3Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkAnalogSensor#getPosition() SparkAnalogSensor.getPosition()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig analogPositionAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_3.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkFlexExternalEncoder#getVelocity()
   * SparkFlexExternalEncoder.getVelocity()} or {@link
   * com.revrobotics.spark.SparkMaxAlternateEncoder#getVelocity()
   * SparkMaxAlternateEncoder.getVelocity()}. The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig externalOrAltEncoderVelocity(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus4Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkFlexExternalEncoder#getVelocity()
   * SparkFlexExternalEncoder.getVelocity()} or {@link
   * com.revrobotics.spark.SparkMaxAlternateEncoder#getVelocity()
   * SparkMaxAlternateEncoder.getVelocity()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig externalOrAltEncoderVelocityAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_4.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkFlexExternalEncoder#getPosition()
   * SparkFlexExternalEncoder.getPosition()} or {@link
   * com.revrobotics.spark.SparkMaxAlternateEncoder#getPosition()
   * SparkMaxAlternateEncoder.getPosition()}. The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig externalOrAltEncoderPosition(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus4Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkFlexExternalEncoder#getPosition()
   * SparkFlexExternalEncoder.getPosition()} or {@link
   * com.revrobotics.spark.SparkMaxAlternateEncoder#getPosition()
   * SparkMaxAlternateEncoder.getPosition()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig externalOrAltEncoderPositionAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_4.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkAbsoluteEncoder#getVelocity() SparkAbsoluteEncoder.getVelocity()}.
   * The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig absoluteEncoderVelocityPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus5Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkAbsoluteEncoder#getVelocity() SparkAbsoluteEncoder.getVelocity()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig absoluteEncoderVelocityAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_5.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkAbsoluteEncoder#getPosition() SparkAbsoluteEncoder.getPosition()}.
   * The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig absoluteEncoderPositionPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus5Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkAbsoluteEncoder#getPosition() SparkAbsoluteEncoder.getPosition()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig absoluteEncoderPositionAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_5.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal representing the unadjusted
   * duty cycle period. The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig unadjustedDutyCyclePeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus6Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal representing the
   * unadjusted duty cycle period.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig unadjustedDutyCyclePeriodAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_6.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal representing the duty cycle
   * period. The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig dutyCyclePeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus6Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal representing the duty
   * cycle period.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig dutyCyclePeriodAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_6.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#getIAccum()
   * SparkClosedLoopController.getIAccum()}. The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig iAccumulationPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus7Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#getIAccum()
   * SparkClosedLoopController.getIAccum()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig iAccumulationAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_7.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#getSetpoint()
   * SparkClosedLoopController.getSetpoint()}. The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig setpointPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus8Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#getSetpoint()
   * SparkClosedLoopController.getSetpoint()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig setSetpointAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_8.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#isAtSetpoint()
   * SparkClosedLoopController.isAtSetpoint()}. The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig isAtSetpointPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus8Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#isAtSetpoint()
   * SparkClosedLoopController.isAtSetpoint()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig isAtSetpointAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_8.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#getSelectedSlot()
   * SparkClosedLoopController.getSelectedSlot()}. The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig selectedSlotPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus8Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#getSelectedSlot()
   * SparkClosedLoopController.getSelectedSlot()}.
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig selectedSlotAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_8.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#getMAXMotionSetpointPosition()
   * SparkClosedLoopController.getMAXMotionSetpointPosition()}. The default period is 100ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig maxMotionSetpointPositionPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus9Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#getMAXMotionSetpointPosition()
   * SparkClosedLoopController.getMAXMotionSetpointPosition()}
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig maxMotionSetpointPositionAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_9.value, enabled);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#getMAXMotionSetpointVelocity()
   * SparkClosedLoopController.getMAXMotionSetpointVelocity()}. The default period is 100ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig maxMotionSetpointVelocityPeriodMs(int periodMs) {
    setPeriodMsCore(SparkParameters.kStatus9Period.value, periodMs);
    return this;
  }

  /**
   * Set whether to always enable the status frame that provides the signal returned by {@link
   * com.revrobotics.spark.SparkClosedLoopController#getMAXMotionSetpointVelocity()
   * SparkClosedLoopController.getMAXMotionSetpointVelocity()}
   *
   * <p>Status frames are only enabled when a signal is requested via its respective getter method,
   * and there may be a small period of time where the signal's data is unavailable due to waiting
   * for the SPARK to receive the command to enable the status frame. Use this method to enable the
   * status frame at all times.
   *
   * <p>If multiple alwaysOn values are set for signals within the same status frame, the result
   * from OR'ing the values will be used.
   *
   * @param enabled True to always enable the status frame
   * @return The modified {@link SignalsConfig} object for method chaining
   */
  public SignalsConfig maxMotionSetpointVelocityAlwaysOn(boolean enabled) {
    setAlwaysOnCore(SparkParameters.kForceEnableStatus_9.value, enabled);
    return this;
  }
}
