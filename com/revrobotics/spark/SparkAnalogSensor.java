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

import com.revrobotics.AnalogInput;
import com.revrobotics.jni.CANSparkJNI;

/** Get an instance of this class by using {@link SparkBase#getAnalog()}. */
public class SparkAnalogSensor implements AnalogInput {
  // package-private to the revrobotics package
  final SparkBase spark;

  // package-private (can only be used by other classes in this package)
  SparkAnalogSensor(SparkBase sparkMax) {
    this.spark = sparkMax;
  }

  /**
   * Get the voltage of the analog sensor.
   *
   * @return Voltage of the sensor.
   */
  @Override
  public double getVoltage() {
    spark.throwIfClosed();
    return CANSparkJNI.c_Spark_GetAnalogVoltage(spark.sparkHandle);
  }

  /**
   * Get the position of the sensor. Returns value in the native unit of 'volt' by default, and can
   * be changed by a scale factor using {@link
   * com.revrobotics.spark.config.AnalogSensorConfig#positionConversionFactor(double)}.
   *
   * @return Position of the sensor
   */
  @Override
  public double getPosition() {
    spark.throwIfClosed();
    return CANSparkJNI.c_Spark_GetAnalogPosition(spark.sparkHandle);
  }

  /**
   * Get the velocity of the sensor. Returns value in the native units of 'volts per second' by
   * default, and can * be changed by a scale factor using {@link
   * com.revrobotics.spark.config.AnalogSensorConfig#velocityConversionFactor(double)}.
   *
   * @return Velocity of the sensor in volts per second
   */
  public double getVelocity() {
    spark.throwIfClosed();
    return CANSparkJNI.c_Spark_GetAnalogVelocity(spark.sparkHandle);
  }
}
