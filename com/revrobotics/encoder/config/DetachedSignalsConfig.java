/*
 * Copyright (c) 2024-2026 REV Robotics
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

public class DetachedSignalsConfig extends BaseConfig {
  /** Create a new object to configure Signals. */
  public DetachedSignalsConfig() {
    super(CANType.kDetachedEncoder);
  }

  private void setPeriodMsCore(int parameterId, int periodMs) {
    Object value = getParameter(parameterId);

    if (value == null) {
      putParameter(parameterId, periodMs);
    } else {
      int currentPeriodMs = (int) value;
      putParameter(parameterId, Math.min(currentPeriodMs, periodMs));
    }
  }

  /**
   * Applies settings from another {@link DetachedSignalsConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The {@link DetachedSignalsConfig} to copy settings from
   * @return The updated {@link DetachedSignalsConfig} for method chaining
   */
  public DetachedSignalsConfig apply(DetachedSignalsConfig config) {
    super.apply(config);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.encoder.DetachedEncoder#getPosition()}. The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link DetachedSignalsConfig} object for method chaining
   */
  public DetachedSignalsConfig encoderPositionPeriodMs(int periodMs) {
    setPeriodMsCore(DetachedEncoderParameter.kStatus3Period.getIndex(), periodMs);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.encoder.DetachedEncoder#getVelocity()}. The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link DetachedSignalsConfig} object for method chaining
   */
  public DetachedSignalsConfig encoderVelocityPeriodMs(int periodMs) {
    setPeriodMsCore(DetachedEncoderParameter.kStatus4Period.getIndex(), periodMs);
    return this;
  }

  /**
   * Set the period (ms) of the status frame that provides the signal returned by {@link
   * com.revrobotics.encoder.DetachedEncoder#getAngle()}. The default period is 20ms.
   *
   * <p>If multiple periods are set for signals within the same status frame, the minimum given
   * value will be used.
   *
   * @param periodMs The period in milliseconds
   * @return The modified {@link DetachedSignalsConfig} object for method chaining
   */
  public DetachedSignalsConfig encoderAnglePeriodMs(int periodMs) {
    setPeriodMsCore(DetachedEncoderParameter.kStatus2Period.getIndex(), periodMs);
    return this;
  }
}
