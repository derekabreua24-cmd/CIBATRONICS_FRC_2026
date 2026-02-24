/*
 * Copyright (c) 2018-2024 REV Robotics
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

package com.revrobotics.spark;

import com.revrobotics.jni.CANSparkJNI;

/**
 * Get an instance of this class by using {@link SparkBase#getForwardLimitSwitch()} or {@link
 * SparkBase#getReverseLimitSwitch()}.
 */
public class SparkLimitSwitch {
  // package-private (can only be used by other classes in this package)
  enum Direction {
    kForward(0),
    kReverse(1);

    @SuppressWarnings("MemberName")
    public final int value;

    Direction(int value) {
      this.value = value;
    }

    public static Direction fromId(int id) {
      switch (id) {
        case 1:
          return kReverse;
        default:
          return kForward;
      }
    }
  }

  private final SparkBase m_device;
  private final Direction m_limitSwitch;

  // package-private (can only be used by other classes in this package)
  SparkLimitSwitch(SparkBase device, Direction direction) {
    m_device = device;
    m_limitSwitch = direction;

    if (direction == null) {
      throw new IllegalArgumentException("limitSwitch must not be null");
    }

    // let the driver check if sim is running and create sim if necessary
    if (direction == Direction.kForward) {
      CANSparkJNI.c_Spark_CreateForwardLimitSwitchSim(device.sparkHandle);
    } else {
      CANSparkJNI.c_Spark_CreateReverseLimitSwitchSim(device.sparkHandle);
    }
  }

  /**
   * Returns {@code true} if the limit switch is pressed, based on the selected polarity.
   *
   * <p>This method works even if the limit switch is not enabled for controller shutdown.
   *
   * @return {@code true} if the limit switch is pressed
   */
  public boolean isPressed() {
    m_device.throwIfClosed();
    return CANSparkJNI.c_Spark_GetLimitSwitch(m_device.sparkHandle, m_limitSwitch.value);
  }
}
