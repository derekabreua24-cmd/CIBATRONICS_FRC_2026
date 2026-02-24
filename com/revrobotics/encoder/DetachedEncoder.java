/*
 * Copyright (c) 2025-2026 REV Robotics
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

package com.revrobotics.encoder;

import com.revrobotics.NativeResourceCleaner;
import com.revrobotics.REVDevice;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.encoder.config.DetachedEncoderAccessor;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.jni.DetachedEncoderJNI;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class DetachedEncoder extends NativeResourceCleaner
    implements REVDevice, RelativeEncoder {
  protected long handle;
  private final int deviceId;
  private final AtomicBoolean isClosed = new AtomicBoolean(false);
  private final Model model;

  /**
   * Accessor for Detached encoder parameter values. This object contains fields and methods to
   * retrieve parameters that have been applied to the device. To set parameters, see {@link
   * DetachedEncoderConfig} and {@link #configure(DetachedEncoderConfig,
   * com.revrobotics.ResetMode)}.
   *
   * <p>NOTE: This uses calls that are blocking to retrieve parameters and should be used
   * infrequently.
   */
  public final DetachedEncoderAccessor detachedEncoderAccessor;

  /**
   * Create a new object to control a Detached Encoder
   *
   * @param id The device ID.
   * @param model The specific model of detached encoder
   */
  public DetachedEncoder(int id, Model model) {
    this.deviceId = id;
    this.model = model;

    if (DetachedEncoderJNI.registerId(deviceId) == REVLibError.kDuplicateCANId.value) {
      throw new IllegalStateException(
          "A DetachedEncoder instance has already been created with this device ID: " + deviceId);
    }
    handle = DetachedEncoderJNI.create(id, model.ordinal());
    registerCleaner(handle);

    detachedEncoderAccessor = new DetachedEncoderAccessor(handle);
  }

  /**
   * Get the configured Device ID of the Detached encoder.
   *
   * @return int device ID
   */
  public int getDeviceId() {
    throwIfClosed();
    return deviceId;
  }

  /**
   * Get the Model of this Detached Encoder Device. Useful for determining if this is a MAXSpline,
   * or other device
   *
   * @return the model of this encoder
   */
  public Model getModel() {
    return model;
  }

  /**
   * Get the position of the encoder. This returns the native units of 'rotations' by default, and
   * can be changed by a scale factor using {@link
   * DetachedEncoderConfig#positionConversionFactor(float)}.
   *
   * @return Number of rotations of the encoder
   */
  @Override
  public double getPosition() {
    throwIfClosed();
    return DetachedEncoderJNI.getEncoderPosition(handle);
  }

  /**
   * Get the absolute position of the encoder. This returns the native units of 'rotations' [0, 1)
   * by default, and can be changed by a scale factor using {@link
   * DetachedEncoderConfig#positionConversionFactor(float)}.
   *
   * @return Number of rotations of the encoder
   */
  public double getAngle() {
    throwIfClosed();
    return DetachedEncoderJNI.getEncoderAngle(handle);
  }

  /**
   * Get the absolute position of the encoder. This returns the native units of 'rotations' [0, 1)
   * without scaling from conversion factors.
   *
   * @return Number of rotations of the encoder
   */
  public double getRawAngle() {
    throwIfClosed();
    return DetachedEncoderJNI.getEncoderRawAngle(handle);
  }

  /**
   * Get the velocity of the encoder. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using {@link DetachedEncoderConfig#velocityConversionFactor(float)}.
   *
   * @return Number the RPM of the encoder
   */
  @Override
  public double getVelocity() {
    throwIfClosed();
    return DetachedEncoderJNI.getEncoderVelocity(handle);
  }

  /**
   * Set the position of the encoder. By default the units are 'rotations' and can be changed by a
   * scale factor using {@link DetachedEncoderConfig#positionConversionFactor(float)}.
   *
   * @param position Number of rotations of the motor
   * @return {@link REVLibError#kOk} if successful
   */
  @Override
  public REVLibError setPosition(double position) {
    throwIfClosed();
    return REVLibError.fromInt(DetachedEncoderJNI.setEncoderPosition(handle, position));
  }

  /**
   * Get the active faults that are currently present on the detached encoder. Faults are fatal
   * errors that prevent the encoder from functioning.
   *
   * @return A struct with each fault and their active value
   */
  public Faults getFaults() {
    throwIfClosed();
    int faultsBitfield = DetachedEncoderJNI.getFaultsBitfield(handle);

    return new Faults(
        (faultsBitfield & 0x1) != 0,
        ((faultsBitfield >> 1) & 0x1) != 0,
        ((faultsBitfield >> 2) & 0x1) != 0,
        ((faultsBitfield >> 3) & 0x1) != 0);
  }

  /**
   * Get the sticky faults that were present on the detached encoder at one point since the sticky
   * faults were last cleared. Faults are fatal errors that prevent the encoder from functioning.
   *
   * <p>Sticky faults can be cleared with {@link DetachedEncoder#clearFaults()}.
   *
   * @return A struct with each fault and their sticky value
   */
  public Faults getStickyFaults() {
    throwIfClosed();
    int faultsBitfield = DetachedEncoderJNI.getStickyFaultsBitfield(handle);

    return new Faults(
        (faultsBitfield & 0x1) != 0,
        ((faultsBitfield >> 1) & 0x1) != 0,
        ((faultsBitfield >> 2) & 0x1) != 0,
        ((faultsBitfield >> 3) & 0x1) != 0);
  }

  /** Clears all sticky faults. */
  public void clearFaults() {
    throwIfClosed();
    DetachedEncoderJNI.clearFaults(handle);
  }

  /**
   * Get the firmware version of the detached encoder.
   *
   * @return Firmware version object
   */
  public FirmwareVersion getFirmwareVersion() {
    throwIfClosed();
    return DetachedEncoderJNI.getFirmwareVersion(handle);
  }

  /**
   * Set the configuration for the Detached encoder.
   *
   * <p>If {@code resetMode} is {com.revrobotics.ResetMode#kResetSafeParameters}, this method will
   * reset safe writable parameters to their default values before setting the given configuration.
   *
   * @param config The desired Detached encoder configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   */
  public void configure(DetachedEncoderConfig config, com.revrobotics.ResetMode resetMode) {
    var configString = config.flatten();

    DetachedEncoderJNI.configure(
        handle, configString, resetMode == com.revrobotics.ResetMode.kResetSafeParameters);
  }

  /**
   * @return periodic status 0, or null if it's not found
   */
  public PeriodicStatus0 getStatus0() {
    throwIfClosed();
    return DetachedEncoderJNI.getStatus0(handle);
  }

  /**
   * @return periodic status 1, or null if it's not found
   */
  public PeriodicStatus1 getStatus1() {
    throwIfClosed();
    return DetachedEncoderJNI.getStatus1(handle);
  }

  /**
   * @return periodic status 2, or null if it's not found
   */
  public PeriodicStatus2 getStatus2() {
    throwIfClosed();
    return DetachedEncoderJNI.getStatus2(handle);
  }

  /**
   * @return periodic status 3, or null if it's not found
   */
  public PeriodicStatus3 getStatus3() {
    throwIfClosed();
    return DetachedEncoderJNI.getStatus3(handle);
  }

  /**
   * @return periodic status 4, or null if it's not found
   */
  public PeriodicStatus4 getStatus4() {
    throwIfClosed();
    return DetachedEncoderJNI.getStatus4(handle);
  }

  /** Closes the Detached encoder controller */
  public void close() {
    if (isClosed.get()) {
      return;
    }
    isClosed.set(true);
    DetachedEncoderJNI.close(handle);
  }

  @Override
  protected OnClean getCleanAction() {
    return DetachedEncoderJNI::destroy;
  }

  private void throwIfClosed() {
    if (isClosed.get()) {
      throw new IllegalStateException("This DetachedEncoder object has previously been closed.");
    }
  }

  public record Faults(boolean unexpected, boolean canTx, boolean canRx, boolean eeprom) {}

  public record FirmwareVersion(
      int major, int minor, int fix, int prerelease, int hardwareMajor, int hardwareMinor) {}

  public enum Model {
    Unknown,
    MAXSplineEncoder,
  }

  public record PeriodicStatus0(int deviceModel) {}

  public record PeriodicStatus1(
      boolean unexpectedFault,
      boolean hasResetFault,
      boolean canTxFault,
      boolean canRxFault,
      boolean eepromFault,
      boolean stickyUnexpectedFault,
      boolean stickyHasResetFault,
      boolean stickyCanTxFault,
      boolean stickyCanRxFault,
      boolean stickyEepromFault) {}

  public record PeriodicStatus2(float rawAngle, float angle) {}

  public record PeriodicStatus3(int timestamp, float position) {}

  public record PeriodicStatus4(int timestamp, float velocity) {}
}
