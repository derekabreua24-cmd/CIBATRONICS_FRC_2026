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

public class SoftLimitConfig extends BaseConfig {
  /** Create a new object to configure the SoftLimits. */
  public SoftLimitConfig() {
    super(CANType.kSpark);
  }

  /**
   * Applies settings from another {@link SoftLimitConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link SoftLimitConfig} to copy settings from
   * @return The updated {@link SoftLimitConfig} for method chaining
   */
  public SoftLimitConfig apply(SoftLimitConfig config) {
    super.apply(config);
    return this;
  }

  /**
   * Set the forward soft limit based on the position of the selected feedback sensor. This will
   * disable motor actuation in the forward direction past this position. This value should have the
   * position conversion factor applied to it.
   *
   * @param limit The forward soft limit position with the conversion factor applied
   * @return The modified {@link SoftLimitConfig} object for method chaining
   */
  public SoftLimitConfig forwardSoftLimit(double limit) {
    putParameter(SparkParameters.kSoftLimitForward.value, (float) limit);
    return this;
  }

  /**
   * Set whether to enable or disable the forward soft limit.
   *
   * @param enabled True to enable the forward soft limit
   * @return The modified {@link SoftLimitConfig} object for method chaining
   */
  public SoftLimitConfig forwardSoftLimitEnabled(boolean enabled) {
    putParameter(SparkParameters.kSoftLimitFwdEn.value, enabled);
    return this;
  }

  /**
   * Set the reverse soft limit based on the position of the selected feedback sensor. This will
   * disable motor actuation in the reverse direction past this position. This value should have the
   * position conversion factor applied to it.
   *
   * @param limit The reverse soft limit position with the conversion factor applied
   * @return The modified {@link SoftLimitConfig} object for method chaining
   */
  public SoftLimitConfig reverseSoftLimit(double limit) {
    putParameter(SparkParameters.kSoftLimitReverse.value, (float) limit);
    return this;
  }

  /**
   * Set whether to enable or disable the reverse soft limit.
   *
   * @param enabled True to enable the reverse soft limit
   * @return The modified {@link SoftLimitConfig} object for method chaining
   */
  public SoftLimitConfig reverseSoftLimitEnabled(boolean enabled) {
    putParameter(SparkParameters.kSoftLimitRevEn.value, enabled);
    return this;
  }
}
