/*
 * Copyright (c) 2025-2026 REV Robotics
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

package com.revrobotics.encoder.config;

import com.revrobotics.config.BaseConfig;

public class DetachedEncoderConfig extends BaseConfig {
  public final DetachedSignalsConfig signals = new DetachedSignalsConfig();

  /** Create a new object to configure the DetachedEncoder */
  public DetachedEncoderConfig() {
    super(CANType.kDetachedEncoder);
  }

  /**
   * Applies settings from another {@link DetachedEncoderConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param other The {@link DetachedEncoderConfig} to copy settings from
   * @return The updated {@link DetachedEncoderConfig} for method chaining
   */
  public DetachedEncoderConfig apply(DetachedEncoderConfig other) {
    super.apply(other);
    this.signals.apply(other.signals);
    return this;
  }

  /**
   * Applies settings from a {@link DetachedSignalsConfig} to this {@link DetachedEncoderConfig}.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link DetachedSignalsConfig} to copy settings from
   * @return The updated {@link DetachedEncoderConfig} for method chaining
   */
  public DetachedEncoderConfig apply(DetachedSignalsConfig config) {
    this.signals.apply(config);
    return this;
  }

  /**
   * Set the phase of the encoder.
   *
   * @param inverted The phase of the encoder
   * @return The modified {@link DetachedEncoderConfig} object for method chaining
   */
  public DetachedEncoderConfig inverted(boolean inverted) {
    putParameter(DetachedEncoderParameter.kEncoderInverted.getIndex(), inverted);
    return this;
  }

  /**
   * Set the conversion factor for the velocity of the encoder. Velocity is returned in native units
   * of rotations per minute and will be multiplied by this conversion factor.
   *
   * @param factor The conversion factor to multiply the native units by
   * @return The modified {@link DetachedEncoderConfig} object for method chaining
   */
  public DetachedEncoderConfig velocityConversionFactor(double factor) {
    putParameter(DetachedEncoderParameter.kVelocityConversionFactor.getIndex(), (float) factor);
    return this;
  }

  /**
   * Set the sampling depth of the velocity calculation process of the encoder. This value sets the
   * number of samples in the average for velocity readings. This value must be in the range [1,
   * 64]. The default value is 64.
   *
   * @param depth The velocity calculation process's sampling depth
   * @return The modified {@link DetachedEncoderConfig} object for method chaining
   */
  public DetachedEncoderConfig averageDepth(int depth) {
    putParameter(DetachedEncoderParameter.kEncoderAverageDepth.getIndex(), depth);
    return this;
  }

  /////////////////////////////////////////////
  // Relative Encoder Specific Settings

  /**
   * Set the conversion factor for the position of the encoder. Position is returned in native units
   * of rotations and will be multiplied by this conversion factor.
   *
   * @param factor The conversion factor to multiply the native units by
   * @return The modified {@link DetachedEncoderConfig} object for method chaining
   */
  public DetachedEncoderConfig positionConversionFactor(double factor) {
    putParameter(DetachedEncoderParameter.kPositionConversionFactor.getIndex(), (float) factor);
    return this;
  }

  /////////////////////////////////////////////
  // Absolute Encoder Specific Settings

  /**
   * Set the conversion factor for the angle of the encoder. Angle is returned in native units of
   * rotations and will be multiplied by this conversion factor.
   *
   * @param factor The conversion factor to multiply the native units by
   * @return The modified {@link DetachedEncoderConfig} object for method chaining
   */
  public DetachedEncoderConfig angleConversionFactor(double factor) {
    putParameter(DetachedEncoderParameter.kAngleConversionFactor.getIndex(), (float) factor);
    return this;
  }

  /**
   * Set whether to enable zero-centering when using the absolute angle. If enabled, the position
   * will be reported in the range (-0.5, 0.5], instead of the default range [0, 1), assuming the
   * default units of rotations.
   *
   * @param isZeroCentered Whether to enable zero centering
   * @return The modified {@link DetachedEncoderConfig} object for method chaining
   */
  public DetachedEncoderConfig dutyCycleZeroCentered(boolean isZeroCentered) {
    putParameter(DetachedEncoderParameter.kDutyCycleZeroCentered.getIndex(), isZeroCentered);
    return this;
  }

  /**
   * Set the zero offset when using the absolute angle, the position that is reported as zero.
   *
   * <p>The zero offset is specified as the reported position of the encoder in the desired zero
   * position as if the zero offset was set to 0, the position conversion factor was set to 1, and
   * inverted was set to false.
   *
   * @param offset The zero offset in the range [0, 1)
   * @return The modified {@link DetachedEncoderConfig} object for method chaining
   */
  public DetachedEncoderConfig dutyCycleOffset(double offset) {
    putParameter(DetachedEncoderParameter.kDutyCycleOffset.getIndex(), (float) offset);
    return this;
  }

  /**
   * Set the length of the start pulse for this encoder. This pulse will be treated as the 0.0
   * position.
   *
   * @param startPulseUs The minimum high pulse in microseconds
   * @return The modified DetachedEncoderConfig object for method chaining
   */
  public DetachedEncoderConfig dutyCycleStartPulseUs(double startPulseUs) {
    putParameter(DetachedEncoderParameter.kDutyCycleStartPulseUs.getIndex(), (float) startPulseUs);
    return this;
  }

  /**
   * Set the length of the end pulse for this encoder. This pulse will be treated as the 1.0
   * position.
   *
   * @param endPulseUs The minimum low pulse in microseconds
   * @return The modified DetachedEncoderConfig object for method chaining
   */
  public DetachedEncoderConfig dutyCycleEndPulseUs(double endPulseUs) {
    putParameter(DetachedEncoderParameter.kDutyCycleEndPulseUs.getIndex(), (float) endPulseUs);
    return this;
  }

  /**
   * Set the expected absolute position signal period in us.
   *
   * @param periodUs The expected absolute position signal period
   * @return The modified DetachedEncoderConfig object for method chaining
   */
  public DetachedEncoderConfig dutyCyclePeriodUs(double periodUs) {
    putParameter(DetachedEncoderParameter.kDutyCyclePeriodUs.getIndex(), (float) periodUs);
    return this;
  }

  @Override
  public String flatten() {
    String flattenedString = "";

    flattenedString += super.flatten();
    flattenedString += signals.flatten();

    return flattenedString;
  }
}
