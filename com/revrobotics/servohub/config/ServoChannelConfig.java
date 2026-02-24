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

import com.revrobotics.config.BaseConfig;
import com.revrobotics.servohub.ServoChannel;

public class ServoChannelConfig extends BaseConfig {
  private ServoChannel.ChannelId channelId;

  public ServoChannelConfig(ServoChannel.ChannelId channelId) {
    super(CANType.kServoHub);
    this.channelId = channelId;
  }

  private static final int[] minPulseParams = {
    com.revrobotics.servohub.config.ServoHubParameter.kChannel0_MinPulseWidth.value,
    ServoHubParameter.kChannel1_MinPulseWidth.value,
    ServoHubParameter.kChannel2_MinPulseWidth.value,
    ServoHubParameter.kChannel3_MinPulseWidth.value,
    ServoHubParameter.kChannel4_MinPulseWidth.value,
    ServoHubParameter.kChannel5_MinPulseWidth.value
  };
  private static final int[] centerPulseParams = {
    ServoHubParameter.kChannel0_CenterPulseWidth.value,
    ServoHubParameter.kChannel1_CenterPulseWidth.value,
    ServoHubParameter.kChannel2_CenterPulseWidth.value,
    ServoHubParameter.kChannel3_CenterPulseWidth.value,
    ServoHubParameter.kChannel4_CenterPulseWidth.value,
    ServoHubParameter.kChannel5_CenterPulseWidth.value
  };
  private static final int[] maxPulseParams = {
    ServoHubParameter.kChannel0_MaxPulseWidth.value,
    ServoHubParameter.kChannel1_MaxPulseWidth.value,
    ServoHubParameter.kChannel2_MaxPulseWidth.value,
    ServoHubParameter.kChannel3_MaxPulseWidth.value,
    ServoHubParameter.kChannel4_MaxPulseWidth.value,
    ServoHubParameter.kChannel5_MaxPulseWidth.value
  };
  private static final int[] disableBehaviorParams = {
    ServoHubParameter.kChannel0_DisableBehavior.value,
    ServoHubParameter.kChannel1_DisableBehavior.value,
    ServoHubParameter.kChannel2_DisableBehavior.value,
    ServoHubParameter.kChannel3_DisableBehavior.value,
    ServoHubParameter.kChannel4_DisableBehavior.value,
    ServoHubParameter.kChannel5_DisableBehavior.value
  };

  /**
   * Applies settings from another {@link ServoChannelConfig} to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this
   *
   * @param config The {@link ServoChannelConfig} to apply settings from
   * @return The updated {@link ServoChannelConfig} for method chaining
   */
  public ServoChannelConfig apply(ServoChannelConfig config) {
    // NOTE: Do Not Call BaseConfig::Apply() as it won't apply the correct
    // parameters. Manually call PutParameter for each parameter present
    // in the input config.

    int srcChanIdx = config.channelId.value;
    int destChanIdx = channelId.value;

    // Only Apply the input config if parameters have been set on it.
    Object minPulse = config.getParameter(minPulseParams[srcChanIdx]);
    if (minPulse != null) {
      putParameter(minPulseParams[destChanIdx], (int) minPulse);
    }
    Object centerPulse = config.getParameter(centerPulseParams[srcChanIdx]);
    if (centerPulse != null) {
      putParameter(centerPulseParams[destChanIdx], (int) centerPulse);
    }
    Object maxPulse = config.getParameter(maxPulseParams[srcChanIdx]);
    if (maxPulse != null) {
      putParameter(maxPulseParams[destChanIdx], (int) maxPulse);
    }

    Object disableBehavior = config.getParameter(disableBehaviorParams[srcChanIdx]);
    if (disableBehavior != null) {
      putParameter(disableBehaviorParams[destChanIdx], (boolean) disableBehavior);
    }

    return this;
  }

  /**
   * Set the min/center/max pulse widths on this channel.
   *
   * @param minPulse_us The minimum pulse width (in us)
   * @param centerPulse_us The center pulse width (in us)
   * @param maxPulse_us The maximum pulse width (in us)
   * @return The modified {@link ServoChannelConfig} object for method chaining
   */
  public ServoChannelConfig pulseRange(int minPulse_us, int centerPulse_us, int maxPulse_us) {
    int chanIdx = channelId.value;

    putParameter(minPulseParams[chanIdx], minPulse_us);
    putParameter(centerPulseParams[chanIdx], centerPulse_us);
    putParameter(maxPulseParams[chanIdx], maxPulse_us);

    return this;
  }

  public static class PulseRange {
    public int minPulse_us;
    public int centerPulse_us;
    public int maxPulse_us;

    public PulseRange(int minPulse, int centerPulse, int maxPulse) {
      this.minPulse_us = minPulse;
      this.centerPulse_us = centerPulse;
      this.maxPulse_us = maxPulse;
    }
  }

  /**
   * Set the min/center/max pulse widths on this channel.
   *
   * @param pulseRange_us The minimum/center/max pulse widths (in microseconds)
   * @return The modified ServoChannelConfig object for method chaining
   */
  public ServoChannelConfig pulseRange(PulseRange pulseRange_us) {
    return this.pulseRange(
        pulseRange_us.minPulse_us, pulseRange_us.centerPulse_us, pulseRange_us.maxPulse_us);
  }

  public enum BehaviorWhenDisabled {
    kDoNotSupplyPower(0),
    kSupplyPower(1);

    @SuppressWarnings("MemberName")
    public final int value;

    BehaviorWhenDisabled(int value) {
      this.value = value;
    }
  };

  /**
   * Set the output power behavior when the channel is disabled.
   *
   * <p>When the channel is enabled [ServoChannel.setEnabled(true)], the output power to the servo
   * follows the channel's power setting [ServoChannel.setPowered()].
   *
   * <p>When the channel is disabled [ServoChannel.setEnabled(false)], the output power to the servo
   * follows the channel's disableBehavior.
   *
   * @param behavior The disable behavior as described above.
   * @return The modified {@link ServoChannelConfig} object for method chaining
   */
  public ServoChannelConfig disableBehavior(BehaviorWhenDisabled behavior) {
    boolean disableBehavior = behavior == BehaviorWhenDisabled.kSupplyPower;

    putParameter(disableBehaviorParams[channelId.value], disableBehavior);

    return this;
  }
}
