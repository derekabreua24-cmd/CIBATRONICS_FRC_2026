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

package com.revrobotics;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

/**
 * Get an instance of this interface by using {@link SparkBase#getEncoder()}, {@link
 * SparkMax#getAlternateEncoder()}, or {@link SparkFlex#getExternalEncoder()}.
 */
public interface RelativeEncoder {
  /**
   * Get the position of the motor. This returns the native units of 'rotations' by default, and can
   * be changed by a scale factor using {@link
   * com.revrobotics.spark.config.AlternateEncoderConfig#positionConversionFactor(double)}, or
   * {@link com.revrobotics.spark.config.ExternalEncoderConfig#positionConversionFactor(double)}, or
   * {@link com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double)}.
   *
   * @return Number of rotations of the motor
   */
  double getPosition();

  /**
   * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using {@link
   * com.revrobotics.spark.config.AlternateEncoderConfig#velocityConversionFactor(double)}, or
   * {@link com.revrobotics.spark.config.ExternalEncoderConfig#velocityConversionFactor(double)}, or
   * {@link com.revrobotics.spark.config.EncoderConfig#velocityConversionFactor(double)}.
   *
   * @return Number the RPM of the motor
   */
  double getVelocity();

  /**
   * Set the position of the encoder. By default the units are 'rotations' and can be changed by a
   * scale factor using {@link
   * com.revrobotics.spark.config.AlternateEncoderConfig#positionConversionFactor(double)}, or
   * {@link com.revrobotics.spark.config.ExternalEncoderConfig#positionConversionFactor(double)}, or
   * {@link com.revrobotics.spark.config.EncoderConfig#positionConversionFactor(double)}.
   *
   * @param position Number of rotations of the motor
   * @return {@link REVLibError#kOk} if successful
   */
  REVLibError setPosition(double position);
}
