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

package com.revrobotics.servohub.config;

import com.revrobotics.servohub.ServoChannel;

public class ServoHubConfigAccessor {
  /**
   * Accessor for parameters relating to servo channel 0. To configure these values, use {@link
   * ServoChannelConfig} and call {@link com.revrobotics.servohub.ServoHub#configure(ServoHubConfig,
   * com.revrobotics.ResetMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final ServoChannelConfigAccessor channel0;

  /**
   * Accessor for parameters relating to servo channel 1. To configure these values, use {@link
   * ServoChannelConfig} and call {@link com.revrobotics.servohub.ServoHub#configure(ServoHubConfig,
   * com.revrobotics.ResetMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final ServoChannelConfigAccessor channel1;

  /**
   * Accessor for parameters relating to servo channel 2. To configure these values, use {@link
   * ServoChannelConfig} and call {@link com.revrobotics.servohub.ServoHub#configure(ServoHubConfig,
   * com.revrobotics.ResetMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final ServoChannelConfigAccessor channel2;

  /**
   * Accessor for parameters relating to servo channel 3. To configure these values, use {@link
   * ServoChannelConfig} and call {@link com.revrobotics.servohub.ServoHub#configure(ServoHubConfig,
   * com.revrobotics.ResetMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final ServoChannelConfigAccessor channel3;

  /**
   * Accessor for parameters relating to servo channel 4. To configure these values, use {@link
   * ServoChannelConfig} and call {@link com.revrobotics.servohub.ServoHub#configure(ServoHubConfig,
   * com.revrobotics.ResetMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final ServoChannelConfigAccessor channel4;

  /**
   * Accessor for parameters relating to servo channel 5. To configure these values, use {@link
   * ServoChannelConfig} and call {@link com.revrobotics.servohub.ServoHub#configure(ServoHubConfig,
   * com.revrobotics.ResetMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final ServoChannelConfigAccessor channel5;

  /**
   * Accessor for parameters relating to the specified servo channel. To configure these values, use
   * ServoChannelConfig and call ServoHub::Configure().
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   *
   * @param channelId The specific channel to access
   * @return The accessor for the specified servo channel
   */
  public ServoChannelConfigAccessor channel(ServoChannel.ChannelId channelId) {
    final ServoChannelConfigAccessor[] sChannelAccessors = {
      channel0, channel1, channel2, channel3, channel4, channel5
    };
    return sChannelAccessors[channelId.value];
  }

  /**
   * Not intended for team use. Only use this if you know what you are doing!
   *
   * @param servoHubHandle
   */
  public ServoHubConfigAccessor(long servoHubHandle) {
    channel0 = new ServoChannelConfigAccessor(ServoChannel.ChannelId.kChannelId0, servoHubHandle);
    channel1 = new ServoChannelConfigAccessor(ServoChannel.ChannelId.kChannelId1, servoHubHandle);
    channel2 = new ServoChannelConfigAccessor(ServoChannel.ChannelId.kChannelId2, servoHubHandle);
    channel3 = new ServoChannelConfigAccessor(ServoChannel.ChannelId.kChannelId3, servoHubHandle);
    channel4 = new ServoChannelConfigAccessor(ServoChannel.ChannelId.kChannelId4, servoHubHandle);
    channel5 = new ServoChannelConfigAccessor(ServoChannel.ChannelId.kChannelId5, servoHubHandle);
  }
}
