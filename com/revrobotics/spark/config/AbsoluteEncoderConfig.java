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
import com.revrobotics.spark.config.SparkMaxConfig.DataPortConfig;

public class AbsoluteEncoderConfig extends BaseConfig {
  public static class Presets {
    /** REV Robotics - Through Bore Encoder */
    public static final AbsoluteEncoderConfig REV_ThroughBoreEncoder =
        new AbsoluteEncoderConfig().startPulseUs(1.0).endPulseUs(1.0);

    /** REV Robotics - Through Bore Encoder V2 */
    public static final AbsoluteEncoderConfig REV_ThroughBoreEncoderV2 =
        new AbsoluteEncoderConfig().startPulseUs(3.88443797).endPulseUs(1.94221899);

    /** REV Robotics - MAXSpline Encoder (via 6-pin JST) */
    public static final AbsoluteEncoderConfig REV_SplineEncoder =
        new AbsoluteEncoderConfig().startPulseUs(1.0).endPulseUs(1.0);
  }

  /** Create a new object to configure an AbsoluteEncoder. */
  public AbsoluteEncoderConfig() {
    super(CANType.kSpark);
  }

  /**
   * Applies settings from another {@link AbsoluteEncoderConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link AbsoluteEncoderConfig} to copy settings from
   * @return The updated {@link AbsoluteEncoderConfig} for method chaining
   */
  public AbsoluteEncoderConfig apply(AbsoluteEncoderConfig config) {
    super.apply(config);
    return this;
  }

  /**
   * Configures the data port to use the absolute encoder, which is specifically required for SPARK
   * MAX.
   *
   * <p>NOTE: This method is only necessary when using an absolute encoder with a SPARK MAX without
   * configuring any of its settings
   *
   * <p>IMPORTANT: SPARK MAX does not support using an absolute encoder in conjunction with an
   * alternate encoder.
   *
   * @return The modified {@link AbsoluteEncoderConfig} object for method chaining
   */
  public AbsoluteEncoderConfig setSparkMaxDataPortConfig() {
    putParameter(
        SparkParameters.kCompatibilityPortConfig.value,
        DataPortConfig.kLimitSwitchesAndAbsoluteEncoder.value);
    return this;
  }

  /**
   * Set the phase of the absolute encoder so that it is in phase with the motor itself.
   *
   * <p>NOTE: This only applies to an encoder used in brushed mode.
   *
   * @param inverted The phase of the encoder
   * @return The modified {@link AbsoluteEncoderConfig} object for method chaining
   */
  public AbsoluteEncoderConfig inverted(boolean inverted) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kDutyCycleInverted.value, inverted);
    return this;
  }

  /**
   * Set the conversion factor for the position of the absolute encoder. Position is returned in
   * native units of rotations and will be multiplied by this conversion factor.
   *
   * @param factor The conversion factor to multiply the native units by
   * @return The modified {@link AbsoluteEncoderConfig} object for method chaining
   */
  public AbsoluteEncoderConfig positionConversionFactor(double factor) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kDutyCyclePositionFactor.value, (float) factor);
    return this;
  }

  /**
   * Set the conversion factor for the velocity of the absolute encoder. Velocity is returned in
   * native units of rotations per minute and will be multiplied by this conversion factor.
   *
   * @param factor The conversion factor to multiply the native units by
   * @return The modified {@link AbsoluteEncoderConfig} object for method chaining
   */
  public AbsoluteEncoderConfig velocityConversionFactor(double factor) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kDutyCycleVelocityFactor.value, (float) factor);
    return this;
  }

  /**
   * Set the zero offset of the absolute encoder, the position that is reported as zero.
   *
   * <p>The zero offset is specified as the reported position of the encoder in the desired zero
   * position as if the zero offset was set to 0, the position conversion factor was set to 1, and
   * inverted was set to false.
   *
   * @param offset The zero offset in the range [0, 1)
   * @return The modified {@link AbsoluteEncoderConfig} object for method chaining
   */
  public AbsoluteEncoderConfig zeroOffset(double offset) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kDutyCycleOffset.value, (float) offset);
    return this;
  }

  /**
   * Set the average sampling depth of the absolute encoder. This is a bit size and should be either
   * 1, 2, 4, 8, 16, 32, 64, or 128 (default).
   *
   * @param depth The average sampling depth of 1, 2, 4, 8, 16, 32, 64, or 128
   * @return The modified {@link AbsoluteEncoderConfig} object for method chaining
   */
  public AbsoluteEncoderConfig averageDepth(int depth) {
    setSparkMaxDataPortConfig();
    int depthIndex;
    switch (depth) {
      case 1:
        depthIndex = 0;
        break;
      case 2:
        depthIndex = 1;
        break;
      case 4:
        depthIndex = 2;
        break;
      case 8:
        depthIndex = 3;
        break;
      case 16:
        depthIndex = 4;
        break;
      case 32:
        depthIndex = 5;
        break;
      case 64:
        depthIndex = 6;
        break;
      default:
        depthIndex = 7;
    }

    putParameter(SparkParameters.kDutyCycleAverageDepth.value, depthIndex);
    return this;
  }

  /**
   * Set the length of the start pulse for this encoder. This pulse will be treated as the 0.0
   * position.
   *
   * @param startPulseUs The minimum high pulse in microseconds
   * @return The modified {@link AbsoluteEncoderConfig} object for method chaining
   */
  public AbsoluteEncoderConfig startPulseUs(double startPulseUs) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kDutyCycleEncoderStartPulseUs.value, (float) startPulseUs);
    return this;
  }

  /**
   * Set the length of the end pulse for this encoder. This pulse will be treated as the 1.0
   * position.
   *
   * @param endPulseUs The minimum low pulse in microseconds
   * @return The modified {@link AbsoluteEncoderConfig} object for method chaining
   */
  public AbsoluteEncoderConfig endPulseUs(double endPulseUs) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kDutyCycleEncoderEndPulseUs.value, (float) endPulseUs);
    return this;
  }

  /**
   * Set whether to enable zero-centering for the absolute encoder. If enabled, the position will be
   * reported in the range (-0.5, 0.5], instead of the default range [0, 1), assuming the default
   * units of rotations.
   *
   * @param zeroCentered Whether to enable zero centering
   * @return The modified {@link AbsoluteEncoderConfig} object for method chaining
   */
  public AbsoluteEncoderConfig zeroCentered(boolean zeroCentered) {
    setSparkMaxDataPortConfig();
    putParameter(SparkParameters.kDutyCycleZeroCentered.value, zeroCentered);
    return this;
  }
}
