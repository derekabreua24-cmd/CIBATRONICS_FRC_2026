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

public class SparkFlexConfig extends SparkBaseConfig {
  public final ExternalEncoderConfig externalEncoder = new ExternalEncoderConfig();

  public static class Presets {
    /** REV Robotics - NEO Brushless Motor V1.1 */
    public static final SparkBaseConfig REV_NEO = new SparkFlexConfig().smartCurrentLimit(60);

    /** REV Robotics - NEO Brushless Motor 2.0 */
    public static final SparkBaseConfig REV_NEO_2 = new SparkFlexConfig().smartCurrentLimit(60);

    /** REV Robotics - NEO 550 Brushless Motor */
    public static final SparkBaseConfig REV_NEO_550 = new SparkFlexConfig().smartCurrentLimit(15);

    /** REV Robotics - NEO Vortex Brushless Motor */
    public static final SparkBaseConfig REV_Vortex = new SparkFlexConfig().smartCurrentLimit(80);

    /* CTRE - Minion: Standalone Brushless Motor */
    public static final SparkBaseConfig CTRE_Minion =
        new SparkFlexConfig().smartCurrentLimit(30).advanceCommutation(120.0);
  }

  /**
   * Applies settings from another {@link SparkFlexConfig} to this one, including all of its nested
   * configurations.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link SparkFlexConfig} to copy settings from
   * @return The updated {@link SparkFlexConfig} for method chaining
   */
  public SparkFlexConfig apply(SparkFlexConfig config) {
    super.apply(config);
    this.externalEncoder.apply(config.externalEncoder);
    return this;
  }

  /**
   * Applies settings from an {@link ExternalEncoderConfig} to this {@link SparkFlexConfig}.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link ExternalEncoderConfig} to copy settings from
   * @return The updated {@link SparkFlexConfig} for method chaining
   */
  public SparkFlexConfig apply(ExternalEncoderConfig config) {
    this.externalEncoder.apply(config);
    return this;
  }

  @Override
  public String flatten() {
    String flattenedString = "";

    flattenedString += super.flatten();
    flattenedString += externalEncoder.flatten();

    return flattenedString;
  }
}
