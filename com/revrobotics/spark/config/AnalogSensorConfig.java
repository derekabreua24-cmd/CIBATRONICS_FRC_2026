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

public class AnalogSensorConfig extends BaseConfig {
  /** Create a new object to configure an AnalogSensor. */
  public AnalogSensorConfig() {
    super(CANType.kSpark);
  }

  /**
   * Applies settings from another {@link AnalogSensorConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link AnalogSensorConfig} to copy settings from
   * @return The updated {@link AnalogSensorConfig} for method chaining
   */
  public AnalogSensorConfig apply(AnalogSensorConfig config) {
    super.apply(config);
    return this;
  }

  /**
   * Set the phase of the analog sensor so that it is in phase with the motor itself.
   *
   * @param inverted The phase of the analog sensor
   * @return The modified {@link AnalogSensorConfig} object for method chaining
   */
  public AnalogSensorConfig inverted(boolean inverted) {
    putParameter(SparkParameters.kAnalogInverted.value, inverted);
    return this;
  }

  /**
   * Set the conversion factor for the position of the analog sensor. Position is returned in native
   * units of volts and will be multiplied by this conversion factor.
   *
   * @param factor The conversion factor to multiply the native units by
   * @return The modified {@link AnalogSensorConfig} object for method chaining
   */
  public AnalogSensorConfig positionConversionFactor(double factor) {
    putParameter(SparkParameters.kAnalogPositionConversion.value, (float) factor);
    return this;
  }

  /**
   * Set the conversion factor for the velocity of the analog sensor. Velocity is returned in native
   * units of volts per second and will be multiplied by this conversion factor.
   *
   * @param factor The conversion factor to multiply the native units by
   * @return The modified {@link AnalogSensorConfig} object for method chaining
   */
  public AnalogSensorConfig velocityConversionFactor(double factor) {
    putParameter(SparkParameters.kAnalogVelocityConversion.value, (float) factor);
    return this;
  }
}
