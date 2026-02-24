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

public class SparkMaxConfig extends SparkBaseConfig {
  public final AlternateEncoderConfig alternateEncoder = new AlternateEncoderConfig();

  public static class Presets {
    /** REV Robotics - NEO Brushless Motor V1.1 */
    public static final SparkBaseConfig REV_NEO = new SparkMaxConfig().smartCurrentLimit(60);

    /** REV Robotics - NEO Brushless Motor 2.0 */
    public static final SparkBaseConfig REV_NEO_2 = new SparkMaxConfig().smartCurrentLimit(60);

    /** REV Robotics - NEO 550 Brushless Motor */
    public static final SparkBaseConfig REV_NEO_550 = new SparkMaxConfig().smartCurrentLimit(15);

    /** REV Robotics - NEO Vortex Brushless Motor */
    public static final SparkBaseConfig REV_Vortex = new SparkMaxConfig().smartCurrentLimit(80);

    /* CTRE - Minion: Standalone Brushless Motor */
    public static final SparkBaseConfig CTRE_Minion =
        new SparkMaxConfig().smartCurrentLimit(30).advanceCommutation(120.0);
  }

  // package-private
  enum DataPortConfig {
    kInvalid(-1),
    kLimitSwitchesAndAbsoluteEncoder(0),
    kAlternateEncoder(1);

    public final int value;

    DataPortConfig(int value) {
      this.value = value;
    }
  }

  private void sanitize() {
    // Check if data port configuration is invalid
    if (getParameter(alternateEncoder, SparkParameters.kCompatibilityPortConfig.value) != null
        && (getParameter(absoluteEncoder, SparkParameters.kCompatibilityPortConfig.value) != null
            || getParameter(limitSwitch, SparkParameters.kCompatibilityPortConfig.value) != null)) {
      removeParameter(alternateEncoder, SparkParameters.kCompatibilityPortConfig.value);
      removeParameter(absoluteEncoder, SparkParameters.kCompatibilityPortConfig.value);
      removeParameter(limitSwitch, SparkParameters.kCompatibilityPortConfig.value);

      // Driver will handle this accordingly
      this.putParameter(
          SparkParameters.kCompatibilityPortConfig.value, DataPortConfig.kInvalid.value);
    }
  }

  /**
   * Applies settings from another {@link SparkMaxConfig} to this one, including all of its nested
   * configurations.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link SparkMaxConfig} to copy settings from
   * @return The updated {@link SparkMaxConfig} for method chaining
   */
  public SparkMaxConfig apply(SparkMaxConfig config) {
    super.apply(config);
    this.alternateEncoder.apply(config.alternateEncoder);
    return this;
  }

  /**
   * Applies settings from an {@link AlternateEncoderConfig} to this {@link SparkMaxConfig}.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link AlternateEncoderConfig} to copy settings from
   * @return The updated {@link SparkMaxConfig} for method chaining
   */
  public SparkMaxConfig apply(AlternateEncoderConfig config) {
    this.alternateEncoder.apply(config);
    return this;
  }

  @Override
  public String flatten() {
    sanitize();

    String flattenedString = "";

    flattenedString += super.flatten();
    flattenedString += alternateEncoder.flatten();

    return flattenedString;
  }
}
