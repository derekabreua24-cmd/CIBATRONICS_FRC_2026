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

public class ServoHubConfig extends BaseConfig {
  public final ServoChannelConfig channel0 =
      new ServoChannelConfig(ServoChannel.ChannelId.kChannelId0);
  public final ServoChannelConfig channel1 =
      new ServoChannelConfig(ServoChannel.ChannelId.kChannelId1);
  public final ServoChannelConfig channel2 =
      new ServoChannelConfig(ServoChannel.ChannelId.kChannelId2);
  public final ServoChannelConfig channel3 =
      new ServoChannelConfig(ServoChannel.ChannelId.kChannelId3);
  public final ServoChannelConfig channel4 =
      new ServoChannelConfig(ServoChannel.ChannelId.kChannelId4);
  public final ServoChannelConfig channel5 =
      new ServoChannelConfig(ServoChannel.ChannelId.kChannelId5);

  public ServoHubConfig() {
    super(CANType.kServoHub);
  }

  /**
   * Applies settings from another ServoHubConfig to this one.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param config The ServoHubConfig to apply settings from
   * @return The updated ServoHubConfig for method chaining
   */
  public ServoHubConfig apply(ServoHubConfig config) {
    super.apply(config);
    channel0.apply(config.channel0);
    channel1.apply(config.channel1);
    channel2.apply(config.channel2);
    channel3.apply(config.channel3);
    channel4.apply(config.channel4);
    channel5.apply(config.channel5);
    return this;
  }

  private final ServoChannelConfig[] channelConfigs = {
    channel0, channel1, channel2, channel3, channel4, channel5
  };

  /**
   * Applies settings from a ServoChannelConfig to this ServoHubConfig.
   *
   * <p>Settings in the provided config will overwrite existing values in this object. Settings not
   * specified in the provided config remain unchanged.
   *
   * @param channelId The channel to apply the settings to
   * @param config The ServoChannelConfig to apply settings from
   * @return The updated ServoHubConfig for method chaining
   */
  public ServoHubConfig apply(ServoChannel.ChannelId channelId, ServoChannelConfig config) {
    channelConfigs[channelId.value].apply(config);

    return this;
  }

  @Override
  public String flatten() {
    String flattenedString =
        super.flatten()
            + channel0.flatten()
            + channel1.flatten()
            + channel2.flatten()
            + channel3.flatten()
            + channel4.flatten()
            + channel5.flatten();

    return flattenedString;
  }
}
