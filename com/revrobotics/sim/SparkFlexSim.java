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

package com.revrobotics.sim;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;

public class SparkFlexSim extends SparkSim {
  private SparkFlex m_spark;

  /**
   * Create a simulated CAN Spark Flex object. This class simulates some of the internal behavior of
   * the device. This class is not required to display to the sim GUI, but is required to interact
   * with it or inject physics simulation.
   *
   * <p>See {@link SparkSim#iterate} for more information on physics simulation.
   *
   * @param sparkFlex The Spark to simulate
   * @param motor The WPILib DCMotor class object to use for calculations. If multiple motors are
   *     connected to the same gearbox and follow each other, a single DCMotor and CANSparkSim can
   *     be used to represent all of them.
   */
  public SparkFlexSim(SparkFlex sparkFlex, DCMotor motor) {
    super(sparkFlex, motor);
    m_spark = sparkFlex;
  }

  /**
   * Get the {@link SparkFlexExternalEncoderSim} object associated with this Spark Device. This will
   * allow you to read/write data from the simulated sensor and view it in the Sim GUI.
   *
   * @return The {@link SparkFlexExternalEncoderSim} object associated with this Spark Device
   */
  public SparkFlexExternalEncoderSim getExternalEncoderSim() {
    return new SparkFlexExternalEncoderSim(m_spark);
  }
}
