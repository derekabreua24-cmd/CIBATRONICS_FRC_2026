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

public class ExternalEncoderConfig extends BaseConfig {
  public static class Presets {
    /** REV Robotics - Through Bore Encoder */
    public static final ExternalEncoderConfig REV_ThroughBoreEncoder =
        new ExternalEncoderConfig().countsPerRevolution(8192);

    /** REV Robotics - Through Bore Encoder V2 */
    public static final ExternalEncoderConfig REV_ThroughBoreEncoderV2 =
        new ExternalEncoderConfig().countsPerRevolution(8192);

    /** REV Robotics - MAXSpline Encoder (via 6-pin JST) */
    public static final ExternalEncoderConfig REV_SplineEncoder =
        new ExternalEncoderConfig().countsPerRevolution(8192);
  }

  public enum Type {
    kQuadrature(0);

    @SuppressWarnings("MemberName")
    public final int value;

    Type(int value) {
      this.value = value;
    }

    public static Type fromId(int id) {
      return kQuadrature;
    }
  }

  /** Create a new object to configure an ExternalEncoder. */
  public ExternalEncoderConfig() {
    super(CANType.kSpark);
  }

  /**
   * Applies settings from another {@link ExternalEncoderConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link ExternalEncoderConfig} to copy settings from
   * @return The updated {@link ExternalEncoderConfig} for method chaining
   */
  public ExternalEncoderConfig apply(ExternalEncoderConfig config) {
    super.apply(config);
    return this;
  }

  /**
   * Set the counts per revolutions of the external encoder.
   *
   * @param cpr The counts per rotation
   * @return The modified {@link ExternalEncoderConfig} object for method chaining
   */
  public ExternalEncoderConfig countsPerRevolution(int cpr) {
    putParameter(SparkParameters.kAltEncoderCountsPerRev.value, cpr);
    return this;
  }

  /**
   * Set the phase of the external encoder so that it is in phase with the motor itself.
   *
   * @param inverted The phase of the encoder
   * @return The modified {@link ExternalEncoderConfig} object for method chaining
   */
  public ExternalEncoderConfig inverted(boolean inverted) {
    putParameter(SparkParameters.kAltEncoderInverted.value, inverted);
    return this;
  }

  /**
   * Set the conversion factor for the position of the external encoder. Position is returned in
   * native units of rotations and will be multiplied by this conversion factor.
   *
   * @param factor The conversion factor to multiply the native units by
   * @return The modified {@link ExternalEncoderConfig} object for method chaining
   */
  public ExternalEncoderConfig positionConversionFactor(double factor) {
    putParameter(SparkParameters.kAltEncoderPositionConversion.value, (float) factor);
    return this;
  }

  /**
   * Set the conversion factor for the velocity of the external encoder. Velocity is returned in
   * native units of rotations per minute and will be multiplied by this conversion factor.
   *
   * @param factor The conversion factor to multiply the native units by
   * @return The modified {@link ExternalEncoderConfig} object for method chaining
   */
  public ExternalEncoderConfig velocityConversionFactor(double factor) {
    putParameter(SparkParameters.kAltEncoderVelocityConversion.value, (float) factor);
    return this;
  }

  /**
   * Set the sampling depth of the velocity calculation process of the external encoder. This value
   * sets the number of samples in the average for velocity readings. For a quadrature encoder, this
   * can be any value from 1 to 64 (default).
   *
   * @param depth The velocity calculation process's sampling depth
   * @return The modified {@link ExternalEncoderConfig} object for method chaining
   */
  public ExternalEncoderConfig averageDepth(int depth) {
    putParameter(SparkParameters.kAltEncoderAverageDepth.value, depth);
    return this;
  }

  /**
   * Set the position measurement period used to calculate the velocity of the external encoder. For
   * a quadrature encoder, this number may be between 1 and 100 (default).
   *
   * <p>The basic formula to calculate velocity is change in position / change in time. This
   * parameter sets the change in time for measurement.
   *
   * @param periodMs Measurement period in milliseconds
   * @return The modified {@link ExternalEncoderConfig} object for method chaining
   */
  public ExternalEncoderConfig measurementPeriod(int periodMs) {
    putParameter(SparkParameters.kAltEncoderSampleDelta.value, periodMs);
    return this;
  }
}
