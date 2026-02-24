/*
 * Copyright (c) 2025 REV Robotics
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

package com.revrobotics.util;

import com.revrobotics.jni.StatusLoggerJNI;

public class StatusLogger {
  /**
   * Manually start capturing data from REV devices to a REV binary log (.revlog).
   *
   * <p><b>Note:</b> This method provides an explicit override for the default logging behavior. In
   * most cases, this is not required, as logging begins automatically on the first call to any
   * REVLib function.
   *
   * <p>Use this method only for advanced cases where logging must begin at a precise moment, such
   * as before the main robot loop or other subsystems are initialized.
   *
   * @see #stop()
   * @see #disableAutoLogging()
   */
  public static void start() {
    StatusLoggerJNI.start();
  }

  /**
   * Stops the data logging session.
   *
   * <p>While stopped, no new data from REV devices will be written to the REV binary log (.revlog).
   * The logging session is not terminated and can be resumed at any time by calling {@link
   * #start()}.
   *
   * @see #start()
   * @see #disableAutoLogging()
   */
  public static void stop() {
    StatusLoggerJNI.stop();
  }

  /**
   * Prevents the data logger from starting automatically.
   *
   * <p><b>IMPORTANT:</b> This method must be called before any other REVLib function is invoked.
   * The recommended placement is as the first line in your {@code robotInit()} method.
   *
   * <p>After calling this, logging will not occur until it is explicitly started with {@link
   * #start()}. This is useful for applications that require full manual control over the logging
   * lifecycle.
   *
   * @see #start()
   * @see #stop()
   */
  public static void disableAutoLogging() {
    StatusLoggerJNI.disableAutoLogging();
  }
}
