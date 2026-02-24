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

package com.revrobotics.spark;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfigAccessor;
import edu.wpi.first.wpilibj.DriverStation;

public class SparkFlex extends SparkBase {
  private SparkFlexExternalEncoder extEncoder;
  private final Object extEncoderLock = new Object();

  /**
   * Accessor for SPARK parameter values. This object contains fields and methods to retrieve
   * parameters that have been applied to the device. To set parameters, see {@link SparkBaseConfig}
   * and {@link SparkBase#configure(SparkBaseConfig, com.revrobotics.ResetMode,
   * com.revrobotics.PersistMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final SparkFlexConfigAccessor configAccessor;

  /**
   * Create a new object to control a SPARK Flex motor Controller
   *
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected
   *     to their matching colors and the hall sensor must be plugged in. Brushed motors must be
   *     connected to the Red and Black terminals only.
   */
  public SparkFlex(int deviceId, MotorType type) {
    super(deviceId, type, SparkModel.SparkFlex);
    configAccessor = new SparkFlexConfigAccessor(sparkHandle);

    if (CANSparkJNI.c_Spark_GetSparkModel(sparkHandle) != SparkModel.SparkFlex.id) {
      DriverStation.reportWarning(
          "CANSparkFlex object created for CAN ID "
              + deviceId
              + ", which is not a SPARK Flex. Some functionalities may not work.",
          true);
    }
  }

  /** ***** Extended Functions ****** */
  /**
   * Returns an object for interfacing with an external quadrature encoder
   *
   * @return An object for interfacing with an external quadrature encoder
   */
  public RelativeEncoder getExternalEncoder() {
    throwIfClosed();
    synchronized (extEncoderLock) {
      if (extEncoder == null) {
        extEncoder = new SparkFlexExternalEncoder(this);
      }
      return extEncoder;
    }
  }
}
