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

/*
 * This file is auto-generated. Do NOT modify it directly.
 * See https://github.com/REVrobotics/SparkParameters
 */

package com.revrobotics.encoder.config;

public enum DetachedEncoderParameter {
  kEncoderAverageDepth(0),
  kEncoderInverted(1),
  kPositionConversionFactor(2),
  kVelocityConversionFactor(3),
  kDutyCycleZeroCentered(4),
  kDutyCycleAverageDepth(5),
  kDutyCycleOffset(6),
  kStatus0Period(7),
  kStatus1Period(8),
  kStatus2Period(9),
  kStatus3Period(10),
  kStatus4Period(11),
  kAngleConversionFactor(12),
  kDutyCycleStartPulseUs(13),
  kDutyCycleEndPulseUs(14),
  kDutyCyclePeriodUs(15);

  @SuppressWarnings("MemberName")
  public final int index;

  DetachedEncoderParameter(int index) {
    this.index = index;
  }

  public int getIndex() {
    return index;
  }
}
