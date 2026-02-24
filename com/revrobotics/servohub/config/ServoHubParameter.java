/*
 * Copyright (c) 2024 REV Robotics
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

package com.revrobotics.servohub.config;

public enum ServoHubParameter {
  kChannel0_MinPulseWidth(0),
  kChannel0_CenterPulseWidth(1),
  kChannel0_MaxPulseWidth(2),
  kChannel1_MinPulseWidth(3),
  kChannel1_CenterPulseWidth(4),
  kChannel1_MaxPulseWidth(5),
  kChannel2_MinPulseWidth(6),
  kChannel2_CenterPulseWidth(7),
  kChannel2_MaxPulseWidth(8),
  kChannel3_MinPulseWidth(9),
  kChannel3_CenterPulseWidth(10),
  kChannel3_MaxPulseWidth(11),
  kChannel4_MinPulseWidth(12),
  kChannel4_CenterPulseWidth(13),
  kChannel4_MaxPulseWidth(14),
  kChannel5_MinPulseWidth(15),
  kChannel5_CenterPulseWidth(16),
  kChannel5_MaxPulseWidth(17),
  kChannel0_DisableBehavior(18),
  kChannel1_DisableBehavior(19),
  kChannel2_DisableBehavior(20),
  kChannel3_DisableBehavior(21),
  kChannel4_DisableBehavior(22),
  kChannel5_DisableBehavior(23);

  @SuppressWarnings("MemberName")
  public final int value;

  ServoHubParameter(int value) {
    this.value = value;
  }
}
