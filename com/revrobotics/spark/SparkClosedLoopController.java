/*
 * Copyright (c) 2018-2025 REV Robotics
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

package com.revrobotics.spark;

import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkJNI;

/** Get an instance of this class by using {@link SparkBase#getClosedLoopController()}. */
public class SparkClosedLoopController {
  private final SparkBase spark;
  private SparkBase.ControlType controlType;

  public enum ArbFFUnits {
    kVoltage(0),
    kPercentOut(1);

    @SuppressWarnings("MemberName")
    public final int value;

    ArbFFUnits(int value) {
      this.value = value;
    }

    public static ArbFFUnits fromInt(int value) {
      switch (value) {
        case 0:
          return kVoltage;
        case 1:
          return kPercentOut;
        default:
          return kVoltage;
      }
    }
  }

  // package-private (can only be used by other classes in this package)
  SparkClosedLoopController(SparkBase device) {
    spark = device;
    controlType = SparkBase.ControlType.kDutyCycle; // Default control type
  }

  /**
   * Set the controller setpoint based on the selected control mode.
   *
   * @param setpoint The setpoint to set depending on the control mode. For basic duty cycle control
   *     this should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts)
   *     Velocity Control: Velocity (RPM) Position Control: Position (Rotations) Current Control:
   *     Current (Amps). Native units can be changed using {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#positionConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#positionConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double)} or
   *     {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#velocityConversionFactor(double)}
   * @param ctrl the control type
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setSetpoint(double setpoint, SparkBase.ControlType ctrl) {
    spark.throwIfClosed();
    controlType = ctrl;
    return setSetpoint(setpoint, ctrl, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the controller setpoint based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param setpoint The setpoint to set depending on the control mode. For basic duty cycle control
   *     this should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts)
   *     Velocity Control: Velocity (RPM) Position Control: Position (Rotations) Current Control:
   *     Current (Amps). Native units can be changed using {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#positionConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#positionConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double)} or
   *     {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#velocityConversionFactor(double)}
   * @param ctrl Is the control type to override with
   * @param slot The {@link ClosedLoopSlot} to use
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setSetpoint(double setpoint, SparkBase.ControlType ctrl, ClosedLoopSlot slot) {
    spark.throwIfClosed();
    return setSetpoint(setpoint, ctrl, slot, 0);
  }

  /**
   * Set the controller setpoint based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param setpoint The setpoint to set depending on the control mode. For basic duty cycle control
   *     this should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts)
   *     Velocity Control: Velocity (RPM) Position Control: Position (Rotations) Current Control:
   *     Current (Amps). Native units can be changed using {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#positionConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#positionConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double)} or
   *     {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#velocityConversionFactor(double)}
   * @param ctrl Is the control type to override with
   * @param slot The {@link ClosedLoopSlot} to use
   * @param arbFeedforward A value from which is represented in voltage applied to the motor after
   *     the result of the specified control mode. The units for the parameter is Volts. This value
   *     is set after the control mode, but before any current limits or ramp rates.
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setSetpoint(
      double setpoint, SparkBase.ControlType ctrl, ClosedLoopSlot slot, double arbFeedforward) {
    spark.throwIfClosed();
    return spark.setpointCommand(setpoint, ctrl, slot.value, arbFeedforward);
  }

  /**
   * Set the controller setpoint based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param setpoint The setpoint to set depending on the control mode. For basic duty cycle control
   *     this should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts)
   *     Velocity Control: Velocity (RPM) Position Control: Position (Rotations) Current Control:
   *     Current (Amps). Native units can be changed using {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#positionConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#positionConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double)} or
   *     {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#velocityConversionFactor(double)}
   * @param ctrl Is the control type to override with
   * @param slot The {@link ClosedLoopSlot} to use
   * @param arbFeedforward A value from which is represented in voltage applied to the motor after
   *     the result of the specified control mode. The units for the parameter is Volts. This value
   *     is set after the control mode, but before any current limits or ramp rates.
   * @param arbFFUnits The units the arbitrary feed forward term is in
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setSetpoint(
      double setpoint,
      SparkBase.ControlType ctrl,
      ClosedLoopSlot slot,
      double arbFeedforward,
      ArbFFUnits arbFFUnits) {
    spark.throwIfClosed();
    return spark.setpointCommand(setpoint, ctrl, slot.value, arbFeedforward, arbFFUnits.value);
  }

  /**
   * Set the controller setpoint based on the selected control mode.
   *
   * @param setpoint The setpoint to set depending on the control mode. For basic duty cycle control
   *     this should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts)
   *     Velocity Control: Velocity (RPM) Position Control: Position (Rotations) Current Control:
   *     Current (Amps). Native units can be changed using {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#positionConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#positionConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double)} or
   *     {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#velocityConversionFactor(double)}
   * @param ctrl the control type
   * @return {@link REVLibError#kOk} if successful
   * @deprecated Use {@link #setSetpoint(double, SparkBase.ControlType)} instead
   */
  @Deprecated(forRemoval = true)
  public REVLibError setReference(double setpoint, SparkBase.ControlType ctrl) {
    return setSetpoint(setpoint, ctrl);
  }

  /**
   * Set the controller setpoint based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param setpoint The setpoint to set depending on the control mode. For basic duty cycle control
   *     this should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts)
   *     Velocity Control: Velocity (RPM) Position Control: Position (Rotations) Current Control:
   *     Current (Amps). Native units can be changed using {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#positionConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#positionConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double)} or
   *     {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#velocityConversionFactor(double)}
   * @param ctrl Is the control type to override with
   * @param slot The {@link ClosedLoopSlot} to use
   * @return {@link REVLibError#kOk} if successful
   * @deprecated Use {@link #setSetpoint(double, SparkBase.ControlType, ClosedLoopSlot)} instead
   */
  @Deprecated(forRemoval = true)
  public REVLibError setReference(
      double setpoint, SparkBase.ControlType ctrl, ClosedLoopSlot slot) {
    return setSetpoint(setpoint, ctrl, slot);
  }

  /**
   * Set the controller setpoint based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param setpoint The setpoint to set depending on the control mode. For basic duty cycle control
   *     this should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts)
   *     Velocity Control: Velocity (RPM) Position Control: Position (Rotations) Current Control:
   *     Current (Amps). Native units can be changed using {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#positionConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#positionConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double)} or
   *     {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#velocityConversionFactor(double)}
   * @param ctrl Is the control type to override with
   * @param slot The {@link ClosedLoopSlot} to use
   * @param arbFeedforward A value from which is represented in voltage applied to the motor after
   *     the result of the specified control mode. The units for the parameter is Volts. This value
   *     is set after the control mode, but before any current limits or ramp rates.
   * @return {@link REVLibError#kOk} if successful
   * @deprecated Use {@link #setSetpoint(double, SparkBase.ControlType, ClosedLoopSlot, double)}
   *     instead
   */
  @Deprecated(forRemoval = true)
  public REVLibError setReference(
      double setpoint, SparkBase.ControlType ctrl, ClosedLoopSlot slot, double arbFeedforward) {
    return setSetpoint(setpoint, ctrl, slot, arbFeedforward);
  }

  /**
   * Set the controller setpoint based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param setpoint The setpoint to set depending on the control mode. For basic duty cycle control
   *     this should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts)
   *     Velocity Control: Velocity (RPM) Position Control: Position (Rotations) Current Control:
   *     Current (Amps). Native units can be changed using {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#positionConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#positionConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double)} or
   *     {@link
   *     com.revrobotics.spark.config.AlternateEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link
   *     com.revrobotics.spark.config.ExternalEncoderConfig#velocityConversionFactor(double)}, or
   *     {@link com.revrobotics.spark.config.EncoderConfig#velocityConversionFactor(double)}
   * @param ctrl Is the control type to override with
   * @param slot The {@link ClosedLoopSlot} to use
   * @param arbFeedforward A value from which is represented in voltage applied to the motor after
   *     the result of the specified control mode. The units for the parameter is Volts. This value
   *     is set after the control mode, but before any current limits or ramp rates.
   * @param arbFFUnits The units the arbitrary feed forward term is in
   * @return {@link REVLibError#kOk} if successful
   * @deprecated Use {@link #setSetpoint(double, SparkBase.ControlType, ClosedLoopSlot, double,
   *     ArbFFUnits)} instead
   */
  @Deprecated(forRemoval = true)
  public REVLibError setReference(
      double setpoint,
      SparkBase.ControlType ctrl,
      ClosedLoopSlot slot,
      double arbFeedforward,
      ArbFFUnits arbFFUnits) {
    return setSetpoint(setpoint, ctrl, slot, arbFeedforward, arbFFUnits);
  }

  /**
   * Get the selected control type used when {@link #setReference(double, SparkBase.ControlType)}
   * was last called.
   *
   * @return The selected control type
   */
  public SparkBase.ControlType getControlType() {
    spark.throwIfClosed();
    return controlType;
  }

  /**
   * Set the I accumulator of the closed loop controller. This is useful when wishing to force a
   * reset on the I accumulator of the closed loop controller. You can also preset values to see how
   * it will respond to certain I characteristics
   *
   * <p>To use this function, the controller must be in a closed loop control mode by calling
   * setReference()
   *
   * @param iAccum The value to set the I accumulator to
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setIAccum(double iAccum) {
    spark.throwIfClosed();
    return REVLibError.fromInt(CANSparkJNI.c_Spark_SetIAccum(spark.sparkHandle, (float) iAccum));
  }

  /**
   * Get the I accumulator of the closed loop controller. This is useful when wishing to see what
   * the I accumulator value is to help with PID tuning
   *
   * @return The value of the I accumulator
   */
  public double getIAccum() {
    spark.throwIfClosed();
    return CANSparkJNI.c_Spark_GetIAccum(spark.sparkHandle);
  }

  /**
   * Get the internal setpoint of the closed loop controller.
   *
   * @return The internal setpoint
   */
  public double getSetpoint() {
    spark.throwIfClosed();
    return CANSparkJNI.c_Spark_GetSetpoint(spark.sparkHandle);
  }

  /**
   * Determine if the setpoint has been reached.
   *
   * @return true if the setpoint is reached; false otherwise
   */
  public boolean isAtSetpoint() {
    spark.throwIfClosed();
    return CANSparkJNI.c_Spark_IsAtSetpoint(spark.sparkHandle);
  }

  /**
   * Get the selected closed loop PID slot.
   *
   * @return The selected closed loop PID slot
   */
  public ClosedLoopSlot getSelectedSlot() {
    spark.throwIfClosed();
    return ClosedLoopSlot.fromInt(CANSparkJNI.c_Spark_GetSelectedSlot(spark.sparkHandle));
  }

  /**
   * Get the MAXMotion internal setpoint position.
   *
   * <p>This will be 0 if the controller is not in a MAXMotion control mode.
   *
   * @return The MAXMotion internal setpoint position in rotations or units specified by the
   *     conversion factor.
   */
  public double getMAXMotionSetpointPosition() {
    spark.throwIfClosed();
    return CANSparkJNI.c_Spark_GetMaxMotionSetpointPosition(spark.sparkHandle);
  }

  /**
   * Get the MAXMotion internal setpoint velocity.
   *
   * <p>This will be 0 if the controller is not in a MAXMotion control mode.
   *
   * @return The MAXMotion internal setpoint velocity in rotations per minute or units specified by
   *     the conversion factor.
   */
  public double getMAXMotionSetpointVelocity() {
    spark.throwIfClosed();
    return CANSparkJNI.c_Spark_GetMaxMotionSetpointVelocity(spark.sparkHandle);
  }
}
