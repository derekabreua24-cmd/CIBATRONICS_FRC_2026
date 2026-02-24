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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;

/**
 * Simulate a moving average filter where samples are normally at some fixed rate that is different
 * than the rate provided by the simulation.
 *
 * <p>A sum of sample deltas is stored until the cumulative sample delta is greater than or equal to
 * the sample rate. Two cases are possible:
 *
 * <p>1) The cumulative delta between samples is *greater or equal* to the sample rate. In this
 * case, take a simple linear interpolation between the sample before and after the sample time, and
 * add this new sample to the moving average.
 *
 * <p>2) The cumulative delta between the samples is *less* than the sample rate. In this case,
 * store the last value, and accumulate the delta time.
 */
public class MovingAverageFilterSim {
  // Actual filter
  private final LinearFilter m_filter;

  // Current filter value
  private double m_value = 0.0;

  private final double m_sampleRate;

  // Pair <delta, last sample value>
  private Pair<Double, Double> m_state = new Pair<>(0.0, 0.0);

  /**
   * Create a MovingAverageFilterSim object.
   *
   * @param taps number of samples in moving average filter
   * @param sampleRate sample rate of moving average filter to simulate
   */
  public MovingAverageFilterSim(int taps, double sampleRate) {
    m_sampleRate = sampleRate;
    m_filter = LinearFilter.movingAverage(taps);
  }

  private double lerp(Pair<Double, Double> p1, Pair<Double, Double> p2, double x) {
    if (p2.getFirst() == p1.getFirst()) {
      return p2.getSecond();
    }
    return p1.getSecond()
        + (x - p1.getFirst())
            * ((p2.getSecond() - p1.getSecond()) / (p2.getFirst() - p1.getFirst()));
  }

  /**
   * Put a new measurement into the moving average filter. This will add any number of samples (or
   * none) depending on the time delta provided.
   *
   * @param value new measurement value
   * @param delta time delta between last measurement value
   */
  public void put(double value, double delta) {
    double newDelta = m_state.getFirst() + delta;

    while (newDelta >= m_sampleRate) {
      // put new filter value
      m_value = m_filter.calculate(lerp(m_state, new Pair<>(newDelta, value), m_sampleRate));
      newDelta = newDelta - m_sampleRate;
    }
    m_state = new Pair<>(newDelta, value);
  }

  /**
   * Get the current value of the filter
   *
   * @return filtered value
   */
  public double get() {
    return m_value;
  }
}
