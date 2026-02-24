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
import edu.wpi.first.wpilibj.DriverStation;

public class FeedForwardConfig extends BaseConfig {
  private int kPIDSlotOffset = SparkParameters.kP_1.value - SparkParameters.kP_0.value;
  private int kFFSlotOffset = SparkParameters.kS_1.value - SparkParameters.kS_0.value;

  /** Create a new object to configure Feed Forward. */
  public FeedForwardConfig() {
    super(CANType.kSpark);
  }

  /**
   * Applies settings from another {@link FeedForwardConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link FeedForwardConfig} to copy settings from
   * @return The updated {@link FeedForwardConfig} for method chaining
   */
  public FeedForwardConfig apply(FeedForwardConfig config) {
    super.apply(config);
    return this;
  }

  /**
   * Set the kS Static Gain of the controller.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link FeedForwardConfig#kS(double, ClosedLoopSlot)}.
   *
   * @param kS The kS gain in Volts
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kS(double kS) {
    kS(kS, ClosedLoopSlot.kSlot0);
    return this;
  }

  /**
   * Set the kV Velocity Gain of the controller.
   *
   * <p>This is not applied in Position control mode.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link FeedForwardConfig#kV(double, ClosedLoopSlot)}.
   *
   * @param kV The kV gain in Volts per velocity
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kV(double kV) {
    kV(kV, ClosedLoopSlot.kSlot0);
    return this;
  }

  /**
   * Set the kA Acceleration Gain of the controller.
   *
   * <p>This is only applied in MAXMotion control modes
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link FeedForwardConfig#kA(double, ClosedLoopSlot)}.
   *
   * @param kA The kA gain in Volts per velocity per second
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kA(double kA) {
    kA(kA, ClosedLoopSlot.kSlot0);
    return this;
  }

  /**
   * Set the kG Static Gravity Gain of the controller.
   *
   * <p>This is statically applied, for an elevator or linear mechanism. For an arm or rotary
   * mechanism, see {@link FeedForwardConfig#kCos(double, ClosedLoopSlot)}. Set this to 0 if kCos is
   * being used.
   *
   * <p>This is only applied in Position and MAXMotion Position control modes
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link FeedForwardConfig#kG(double, ClosedLoopSlot)}.
   *
   * @param kG The kG gain in Volts
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kG(double kG) {
    kG(kG, ClosedLoopSlot.kSlot0);
    return this;
  }

  /**
   * Set the kCos Cosine Gravity Gain of the controller.
   *
   * <p>This is multiplied by the cosine of the absolute position of the mechanism (See {@link
   * FeedForwardConfig#kCosRatio(double kCosRatio)} for info on configuring this) for an arm
   * mechanism. Set it to 0 if kG is being used.
   *
   * <p>This is only applied in Position and MAXMotion Position control modes
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link FeedForwardConfig#kCos(double, ClosedLoopSlot)}.
   *
   * @param kCos The kCos gain in Volts
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kCos(double kCos) {
    kCos(kCos, ClosedLoopSlot.kSlot0);
    return this;
  }

  /**
   * Set the kCosRatio of the controller.
   *
   * <p>This sets the ratio that is used to calculate the absolute position of your arm mechanism
   * for use with kCos. This is applied after the conversion factor and should convert from those
   * units to absolute rotations of your mechanism. Ensure your selected encoder is zeroed such that
   * 0 = horizontal.
   *
   * <p>This will set the value for closed loop slot 0. To set the value for a specific closed loop
   * slot, use {@link FeedForwardConfig#kCosRatio(double, ClosedLoopSlot)}.
   *
   * @param kCosRatio The kCosRatio
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kCosRatio(double kCosRatio) {
    kCosRatio(kCosRatio, ClosedLoopSlot.kSlot0);
    return this;
  }

  /**
   * Set the kS Static Gain of the controller for a specific closed loop slot.
   *
   * @param kS The kS gain in Volts
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kS(double kS, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kS_0.value + (slot.value * kFFSlotOffset), (float) kS);
    return this;
  }

  /**
   * Set the kV Velocity Gain of the controller for a specific closed loop slot.
   *
   * <p>This is not applied in Position control mode.
   *
   * @param kV The kV gain in Volts per velocity
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kV(double kV, ClosedLoopSlot slot) {
    // uses PID Slot Offset because it's not lumped with the other feedforward
    // values
    putParameter(SparkParameters.kV_0.value + (slot.value * kPIDSlotOffset), (float) kV);
    return this;
  }

  /**
   * Set the kA Acceleration Gain of the controller for a specific closed loop slot.
   *
   * <p>This is only applied in MAXMotion control modes
   *
   * @param kA The kA gain in Volts per velocity per second
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kA(double kA, ClosedLoopSlot slot) {
    putParameter(SparkParameters.kA_0.value + (slot.value * kFFSlotOffset), (float) kA);
    return this;
  }

  /**
   * Set the kG Static Gravity Gain of the controller for a specific closed loop slot.
   *
   * <p>This is statically applied, for an elevator or linear mechanism. For an arm or rotary
   * mechanism, see {@link FeedForwardConfig#kCos(double, ClosedLoopSlot)}. Set this to 0 if kCos is
   * being used.
   *
   * <p>This is only applied in Position and MAXMotion Position control modes
   *
   * @param kG The kG gain in Volts
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kG(double kG, ClosedLoopSlot slot) {
    if (getParameter(SparkParameters.kCos_0.value + (slot.value * kFFSlotOffset)) != null) {
      // kCos has been set, warn user and do not set kG
      DriverStation.reportWarning(
          "kG is overriden by kCos, only kCos will be used on Closed Loop Slot " + slot.value,
          true);
      return this;
    }

    putParameter(SparkParameters.kG_0.value + (slot.value * kFFSlotOffset), (float) kG);
    return this;
  }

  /**
   * Set the kCos Cosine Gravity Gain of the controller for a specific closed loop slot.
   *
   * <p>This is multiplied by the cosine of the absolute position of the mechanism (See {@link
   * FeedForwardConfig#kCosRatio(double kCosRatio, ClosedLoopSlot slot)} for info on configuring
   * this) for an arm mechanism. Set it to 0 if kG is being used.
   *
   * <p>This is only applied in Position and MAXMotion Position control modes
   *
   * @param kCos The kCos gain in Volts
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kCos(double kCos, ClosedLoopSlot slot) {
    if (getParameter(SparkParameters.kG_0.value + (slot.value * kFFSlotOffset)) != null) {
      // kCos has been set, warn user and do not set kG
      DriverStation.reportWarning(
          "kG is overriden by kCos, only kCos will be used on Closed Loop Slot " + slot.value,
          true);
      removeParameter(SparkParameters.kG_0.value + (slot.value * kFFSlotOffset));
    }

    putParameter(SparkParameters.kCos_0.value + (slot.value * kFFSlotOffset), (float) kCos);
    return this;
  }

  /**
   * Set the kCosRatio of the controller for a specific closed loop slot.
   *
   * <p>This sets the ratio that is used to calculate the absolute position of your arm mechanism
   * for use with kCos. This is applied after the conversion factor and should convert from those
   * units to absolute rotations of your mechanism. Ensure your selected encoder is zeroed such that
   * 0 = horizontal.
   *
   * @param kCosRatio The kCosRatio
   * @param slot The closed loop slot to set the values for
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig kCosRatio(double kCosRatio, ClosedLoopSlot slot) {
    putParameter(
        SparkParameters.kCosRatio_0.value + (slot.value * kFFSlotOffset), (float) kCosRatio);
    return this;
  }

  /**
   * Set the kS and kV gains for slot 0 in one call.
   *
   * <p>For more information on the kS and kV gains, see {@link FeedForwardConfig#kS(double)} and
   * {@link FeedForwardConfig#kV(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kV The kV gain in Volts per velocity
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig sv(double kS, double kV) {
    kS(kS);
    kV(kV);
    return this;
  }

  /**
   * Set the kS, kV, and kA gains for slot 0 in one call.
   *
   * <p>For more information on the kS, kV, and kA gains, see {@link FeedForwardConfig#kS(double)},
   * {@link FeedForwardConfig#kV(double)}, and {@link FeedForwardConfig#kA(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kV The kV gain in Volts per velocity
   * @param kA The kA gain in Volts per velocity per second
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig sva(double kS, double kV, double kA) {
    kS(kS);
    kV(kV);
    kA(kA);
    return this;
  }

  /**
   * Set the kS, kV, kA, and kG gains for slot 0 in one call.
   *
   * <p>For more information on the kS, kV, kA, and kG gains, see {@link
   * FeedForwardConfig#kS(double)}, {@link FeedForwardConfig#kV(double)}, {@link
   * FeedForwardConfig#kA(double)}, and {@link FeedForwardConfig#kG(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kV The kV gain in Volts per velocity
   * @param kA The kA gain in Volts per velocity per second
   * @param kG The kG gain in Volts
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig svag(double kS, double kV, double kA, double kG) {
    kS(kS);
    kV(kV);
    kA(kA);
    kG(kG);
    return this;
  }

  /**
   * Set the kS, kV, kA, kCos, and kCosRatio gains for slot 0 in one call.
   *
   * <p>For more information on the kS, kV, kA, kCos, and kCosRatio gains, see {@link
   * FeedForwardConfig#kS(double)}, {@link FeedForwardConfig#kV(double)}, {@link
   * FeedForwardConfig#kA(double)}, {@link FeedForwardConfig#kCos(double)}, and {@link
   * FeedForwardConfig#kCosRatio(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kV The kV gain in Volts per velocity
   * @param kA The kA gain in Volts per velocity per second
   * @param kCos The kCos gain in Volts
   * @param kCosRatio The ratio used to calculate the absolute position of your arm mechanism for
   *     use with kCos
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig svacr(double kS, double kV, double kA, double kCos, double kCosRatio) {
    kS(kS);
    kV(kV);
    kA(kA);
    kCos(kCos);
    kCosRatio(kCosRatio);
    return this;
  }

  /**
   * Set the kS and kG gains for slot 0 in one call.
   *
   * <p>For more information on the kS and kG gains, see {@link FeedForwardConfig#kS(double)} and
   * {@link FeedForwardConfig#kG(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kG The kG gain in Volts
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig sg(double kS, double kG) {
    kS(kS);
    kG(kG);
    return this;
  }

  /**
   * Set the kS, kCos, and kCosRatio gains for slot 0 in one call.
   *
   * <p>For more information on the kS, kCos, and kCosRatio gains, see {@link
   * FeedForwardConfig#kS(double)}, {@link FeedForwardConfig#kCos(double)}, and {@link
   * FeedForwardConfig#kCosRatio(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kCos The kCos gain in Volts
   * @param kCosRatio The ratio used to calculate the absolute position of your arm mechanism for
   *     use with kCos
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig scr(double kS, double kCos, double kCosRatio) {
    kS(kS);
    kCos(kCos);
    kCosRatio(kCosRatio);
    return this;
  }

  /**
   * Set the kS and kV gains for the provided slot in one call.
   *
   * <p>For more information on the kS and kV gains, see {@link FeedForwardConfig#kS(double)} and
   * {@link FeedForwardConfig#kV(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kV The kV gain in Volts per velocity
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig sv(double kS, double kV, ClosedLoopSlot slot) {
    kS(kS, slot);
    kV(kV, slot);
    return this;
  }

  /**
   * Set the kS, kV, and kA gains for the provided slot in one call.
   *
   * <p>For more information on the kS, kV, and kA gains, see {@link FeedForwardConfig#kS(double)},
   * {@link FeedForwardConfig#kV(double)}, and {@link FeedForwardConfig#kA(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kV The kV gain in Volts per velocity
   * @param kA The kA gain in Volts per velocity per second
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig sva(double kS, double kV, double kA, ClosedLoopSlot slot) {
    kS(kS, slot);
    kV(kV, slot);
    kA(kA, slot);
    return this;
  }

  /**
   * Set the kS, kV, kA, and kG gains for the provided slot in one call.
   *
   * <p>For more information on the kS, kV, kA, and kG gains, see {@link
   * FeedForwardConfig#kS(double)}, {@link FeedForwardConfig#kV(double)}, {@link
   * FeedForwardConfig#kA(double)}, and {@link FeedForwardConfig#kG(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kV The kV gain in Volts per velocity
   * @param kA The kA gain in Volts per velocity per second
   * @param kG The kG gain in Volts
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig svag(double kS, double kV, double kA, double kG, ClosedLoopSlot slot) {
    kS(kS, slot);
    kV(kV, slot);
    kA(kA, slot);
    kG(kG, slot);
    return this;
  }

  /**
   * Set the kS, kV, kA, kCos, and kCosRatio gains for the provided slot in one call.
   *
   * <p>For more information on the kS, kV, kA, kCos, and kCosRatio gains, see {@link
   * FeedForwardConfig#kS(double)}, {@link FeedForwardConfig#kV(double)}, {@link
   * FeedForwardConfig#kA(double)}, {@link FeedForwardConfig#kCos(double)}, and {@link
   * FeedForwardConfig#kCosRatio(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kV The kV gain in Volts per velocity
   * @param kA The kA gain in Volts per velocity per second
   * @param kCos The kCos gain in Volts
   * @param kCosRatio The ratio used to calculate the absolute position of your arm mechanism for
   *     use with kCos
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig svacr(
      double kS, double kV, double kA, double kCos, double kCosRatio, ClosedLoopSlot slot) {
    kS(kS, slot);
    kV(kV, slot);
    kA(kA, slot);
    kCos(kCos, slot);
    kCosRatio(kCosRatio, slot);
    return this;
  }

  /**
   * Set the kS and kG gains for the provided slot in one call.
   *
   * <p>For more information on the kS and kG gains, see {@link FeedForwardConfig#kS(double)} and
   * {@link FeedForwardConfig#kG(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kG The kG gain in Volts
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig sg(double kS, double kG, ClosedLoopSlot slot) {
    kS(kS, slot);
    kG(kG, slot);
    return this;
  }

  /**
   * Set the kS, kCos, and kCosRatio gains for the provided slot in one call.
   *
   * <p>For more information on the kS, kCos, and kCosRatio gains, see {@link
   * FeedForwardConfig#kS(double)}, {@link FeedForwardConfig#kCos(double)}, and {@link
   * FeedForwardConfig#kCosRatio(double)}.
   *
   * @param kS The kS gain in Volts
   * @param kCos The kCos gain in Volts
   * @param kCosRatio The ratio used to calculate the absolute position of your arm mechanism for
   *     use with kCos
   * @return The modified {@link FeedForwardConfig} object for method chaining
   */
  public FeedForwardConfig scr(double kS, double kCos, double kCosRatio, ClosedLoopSlot slot) {
    kS(kS, slot);
    kCos(kCos, slot);
    kCosRatio(kCosRatio, slot);
    return this;
  }
}
