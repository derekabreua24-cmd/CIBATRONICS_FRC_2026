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
import com.revrobotics.spark.ClosedLoopSlot;

public class MAXMotionConfig extends BaseConfig {
  public enum MAXMotionPositionMode {
    kMAXMotionTrapezoidal(0);

    // TODO(rylan): Add when S-curve is supported
    // kMAXMotionSCurve(1);

    @SuppressWarnings("MemberName")
    public final int value;

    MAXMotionPositionMode(int value) {
      this.value = value;
    }

    public static MAXMotionPositionMode fromInt(int value) {
      switch (value) {
        case 0:
          return kMAXMotionTrapezoidal;
          // TODO(rylan): Add when S-curve is supported
          // case 1:
          // return kMAXMotionSCurve;
        default:
          return kMAXMotionTrapezoidal;
      }
    }
  }

  /** Create a new object to configure MAXMotion. */
  public MAXMotionConfig() {
    super(CANType.kSpark);
  }

  /**
   * Applies settings from another {@link MAXMotionConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link MAXMotionConfig} to copy settings from
   * @return The updated {@link MAXMotionConfig} for method chaining
   */
  public MAXMotionConfig apply(MAXMotionConfig config) {
    super.apply(config);
    return this;
  }

  /**
   * Set the cruise velocity for the MAXMotion mode of the controller. Natively, the units are in
   * RPM but will be affected by the velocity conversion factor.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link MAXMotionConfig#cruiseVelocity(double, ClosedLoopSlot)}.
   *
   * @param maxVelocity The maximum velocity with the velocity conversion factor applied
   * @return The modified {@link MAXMotionConfig} object for method chaining
   * @deprecated Use {@link #cruiseVelocity(double)} instead.
   */
  @Deprecated(forRemoval = true)
  public MAXMotionConfig maxVelocity(double maxVelocity) {
    return maxVelocity(maxVelocity, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the cruise velocity for the MAXMotion mode of the controller. Natively, the units are in
   * RPM but will be affected by the velocity conversion factor.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link MAXMotionConfig#cruiseVelocity(double, ClosedLoopSlot)}.
   *
   * @param cruiseVelocity The maximum velocity with the velocity conversion factor applied
   * @return The modified {@link MAXMotionConfig} object for method chaining
   */
  public MAXMotionConfig cruiseVelocity(double cruiseVelocity) {
    return cruiseVelocity(cruiseVelocity, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the cruise velocity for the MAXMotion mode of the controller for a specific closed loop
   * slot. Natively, the units are in RPM but will be affected by the velocity conversion factor.
   *
   * @param maxVelocity The maximum velocity with the velocity conversion factor applied
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link MAXMotionConfig} object for method chaining
   * @deprecated Use {@link #cruiseVelocity(double, ClosedLoopSlot)} instead.
   */
  @Deprecated(forRemoval = true)
  public MAXMotionConfig maxVelocity(double maxVelocity, ClosedLoopSlot slot) {
    putParameter(
        SparkParameters.kMAXMotionCruiseVelocity_0.value + slot.value * 5, (float) maxVelocity);
    return this;
  }

  /**
   * Set the cruise velocity for the MAXMotion mode of the controller for a specific closed loop
   * slot. Natively, the units are in RPM but will be affected by the velocity conversion factor.
   *
   * @param cruiseVelocity The cruise velocity with the velocity conversion factor applied
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link MAXMotionConfig} object for method chaining
   */
  public MAXMotionConfig cruiseVelocity(double cruiseVelocity, ClosedLoopSlot slot) {
    putParameter(
        SparkParameters.kMAXMotionCruiseVelocity_0.value + slot.value * 5, (float) cruiseVelocity);
    return this;
  }

  /**
   * Set the maximum acceleration for the MAXMotion mode of the controller. This is the rate at
   * which the velocity will increase until the max velocity is reached. Natively, the units are in
   * RPM per second but will be affected by the velocity conversion factor.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link MAXMotionConfig#maxAcceleration(double, ClosedLoopSlot)}.
   *
   * @param maxAcceleration The maximum acceleration with the velocity conversion factor applied
   * @return The modified {@link MAXMotionConfig} object for method chaining
   */
  public MAXMotionConfig maxAcceleration(double maxAcceleration) {
    return maxAcceleration(maxAcceleration, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the maximum acceleration for the MAXMotion mode of the controller for a specific closed
   * loop slot. This is the rate at which the velocity will increase until the max velocity is
   * reached. Natively, the units are in RPM per second but will be affected by the velocity
   * conversion factor.
   *
   * @param maxAcceleration The maximum acceleration with the velocity conversion factor applied
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link MAXMotionConfig} object for method chaining
   */
  public MAXMotionConfig maxAcceleration(double maxAcceleration, ClosedLoopSlot slot) {
    putParameter(
        SparkParameters.kMAXMotionMaxAccel_0.value + slot.value * 5, (float) maxAcceleration);
    return this;
  }

  /**
   * Set the allowed profile error for the MAXMotion mode of the controller. This value is how much
   * deviation from the profile is tolerated before the profile is regenerated. Natively, the units
   * are in rotations but will be affected by the position conversion factor.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link MAXMotionConfig#allowedProfileError(double, ClosedLoopSlot)}.
   *
   * @param allowedError The allowed error with the position conversion factor applied
   * @return The modified {@link MAXMotionConfig} object for method chaining
   * @deprecated Use {@link #allowedProfileError(double)} instead.
   */
  @Deprecated(forRemoval = true)
  public MAXMotionConfig allowedClosedLoopError(double allowedError) {
    return allowedClosedLoopError(allowedError, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the allowed profile error for the MAXMotion mode of the controller. This value is how much
   * deviation from the profile is tolerated before the profile is regenerated. Natively, the units
   * are in rotations but will be affected by the position conversion factor.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link MAXMotionConfig#allowedProfileError(double, ClosedLoopSlot)}.
   *
   * @param allowedError The allowed error with the position conversion factor applied
   * @return The modified {@link MAXMotionConfig} object for method chaining
   */
  public MAXMotionConfig allowedProfileError(double allowedError) {
    return allowedProfileError(allowedError, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the allowed profile error for the MAXMotion mode of the controller for a specific PID slot.
   * This value is how much deviation from the profile is tolerated before the profile is
   * regenerated. Natively, the units are in rotations but will be affected by the position
   * conversion factor.
   *
   * @param allowedError The allowed error with the position conversion factor applied
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link MAXMotionConfig} object for method chaining
   * @deprecated Use {@link #allowedProfileError(double, ClosedLoopSlot)} instead.
   */
  @Deprecated(forRemoval = true)
  public MAXMotionConfig allowedClosedLoopError(double allowedError, ClosedLoopSlot slot) {
    putParameter(
        SparkParameters.kMAXMotionAllowedProfileError_0.value + slot.value * 5,
        (float) allowedError);
    return this;
  }

  /**
   * Set the allowed profile error for the MAXMotion mode of the controller for a specific PID slot.
   * This value is how much deviation from the profile is tolerated before the profile is
   * regenerated. Natively, the units are in rotations but will be affected by the position
   * conversion factor.
   *
   * @param allowedError The allowed error with the position conversion factor applied
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link MAXMotionConfig} object for method chaining
   */
  public MAXMotionConfig allowedProfileError(double allowedError, ClosedLoopSlot slot) {
    putParameter(
        SparkParameters.kMAXMotionAllowedProfileError_0.value + slot.value * 5,
        (float) allowedError);
    return this;
  }

  /**
   * Set the MAXMotion position control mode of the controller.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link MAXMotionConfig#positionMode(MAXMotionPositionMode, ClosedLoopSlot)}.
   *
   * @param mode The MAXmotion position mode
   * @return The modified {@link MAXMotionConfig} object for method chaining
   */
  public MAXMotionConfig positionMode(MAXMotionPositionMode mode) {
    return positionMode(mode, ClosedLoopSlot.kSlot0);
  }

  /**
   * Set the MAXMotion position control mode of the controller for a specific closed loop slot.
   *
   * @param mode The MAXmotion position mode
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link MAXMotionConfig} object for method chaining
   */
  public MAXMotionConfig positionMode(MAXMotionPositionMode mode, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kMAXMotionPositionMode_0.value + slot.value * 5, mode.value);
    return this;
  }
}
