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

package com.revrobotics.spark;

import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.sim.*;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SparkSim {
  private final SimDouble m_appliedOutput;
  private final SimDouble m_velocity;
  private final SimDouble m_position;
  private final SimDouble m_busVoltage;
  private final SimDouble m_motorCurrent;
  private final SimDouble m_setpoint;
  private final SimDouble m_arbFF;
  private final SimInt m_closedLoopSlot;
  private final SimInt m_arbFFUnits;
  private final SparkBase m_spark;
  private final DCMotor m_dcMotor;
  private final SimInt m_controlMode;
  private final MovingAverageFilterSim m_velocityAverage = new MovingAverageFilterSim(2, 0.016);
  // private final MovingAverageFilterSim m_velocityAverage = new
  // MovingAverageFilterSim(8, 0.032);
  private Boolean m_enable = null;
  private String m_deviceName;

  /**
   * Create a simulated CAN Spark Max or Flex object. This class simulates some of the internal
   * behavior of the device. CANSparkMaxSim and CANSparkFlexSim wrap this class. This class is not
   * required to display to the sim GUI, but is required to interact with it or inject physics
   * simulation.
   *
   * <p>See {@link SparkSim#iterate} for more information on physics simulation.
   *
   * @param spark The Spark to simulate
   * @param motor The WPILib DCMotor class object to use for calculations. If multiple motors are
   *     connected to the same gearbox and follow each other, a single DCMotor and CANSparkSim can
   *     be used to represent all of them.
   */
  public SparkSim(SparkBase spark, DCMotor motor) {
    String deviceType = "UNKNOWN";
    if (spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
      deviceType = "SPARK Flex";
    } else if (spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
      deviceType = "SPARK MAX";
    }

    m_deviceName = deviceType + " [" + spark.getDeviceId() + "]";
    SimDeviceSim sparkSim = new SimDeviceSim(m_deviceName);
    m_appliedOutput = sparkSim.getDouble("Applied Output");
    m_position = sparkSim.getDouble("Position");
    m_velocity = sparkSim.getDouble("Velocity");
    m_busVoltage = sparkSim.getDouble("Bus Voltage");
    m_motorCurrent = sparkSim.getDouble("Motor Current");
    m_setpoint = sparkSim.getDouble("Setpoint");
    m_arbFF = sparkSim.getDouble("Arbitrary Feedforward");
    m_closedLoopSlot = sparkSim.getInt("Closed Loop Slot");
    m_arbFFUnits = sparkSim.getInt("ArbFF Units");
    m_controlMode = sparkSim.getInt("Control Mode");
    m_spark = spark;
    m_dcMotor = motor;
  }

  /**
   * Get the simulated applied output. This matches the value from the CANSparkBase
   * getAppliedOutput(). Multiply by vbus to get the motor voltage.
   *
   * @return applied output [-1, 1]
   */
  public double getAppliedOutput() {
    return m_appliedOutput.get();
  }

  /**
   * Set the simulated applied output. Use this only in place of iterate().
   *
   * @param appliedOutput simulated applied output value [-1, 1]
   */
  public void setAppliedOutput(double appliedOutput) {
    m_appliedOutput.set(appliedOutput);
  }

  /**
   * Get the setpoint sent to the device
   *
   * @return setpoint value
   */
  public double getSetpoint() {
    return m_setpoint.get();
  }

  /**
   * Get the simulated closed loop slot
   *
   * @return the closed loop slot being used
   */
  public ClosedLoopSlot getClosedLoopSlot() {
    switch (m_closedLoopSlot.get()) {
      case 0:
        return ClosedLoopSlot.kSlot0;
      case 1:
        return ClosedLoopSlot.kSlot1;
      case 2:
        return ClosedLoopSlot.kSlot2;
      case 3:
        return ClosedLoopSlot.kSlot3;
      default:
        return ClosedLoopSlot.kSlot0;
    }
  }

  // return true if limited (i.e. output should be 0 in this direction)
  private boolean runLimitLogic(boolean forward) {
    if (forward) {
      if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
        if (((SparkFlex) m_spark).configAccessor.softLimit.getForwardSoftLimitEnabled()
            && ((SparkFlex) m_spark).configAccessor.softLimit.getForwardSoftLimit()
                < m_position.get()) {
          return true;
        }
      } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
        if (((SparkMax) m_spark).configAccessor.softLimit.getForwardSoftLimitEnabled()
            && ((SparkMax) m_spark).configAccessor.softLimit.getForwardSoftLimit()
                < m_position.get()) {
          return true;
        }
      }

      return (getForwardLimitSwitchSim().getEnabled() && getForwardLimitSwitchSim().getPressed());
    } else {
      if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
        if (((SparkFlex) m_spark).configAccessor.softLimit.getReverseSoftLimitEnabled()
            && ((SparkFlex) m_spark).configAccessor.softLimit.getReverseSoftLimit()
                > m_position.get()) {
          return true;
        }
      } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
        if (((SparkMax) m_spark).configAccessor.softLimit.getReverseSoftLimitEnabled()
            && ((SparkMax) m_spark).configAccessor.softLimit.getReverseSoftLimit()
                > m_position.get()) {
          return true;
        }
      }

      return (getReverseLimitSwitchSim().getEnabled() && getReverseLimitSwitchSim().getPressed());
    }
  }

  /**
   * Run internal calculations and set internal state.
   *
   * <p>This method belongs in Simulation Periodic. Use a WPILib physics simulation class or
   * equivalent to calculate the velocity from the applied output and pass it in to this method,
   * which will update the simulated state of the motor.
   *
   * <p>Simulating a Spark this way will use the configurations and controls of the original
   * CANSparkMax or CANSparkFlex device to simulate velocity noise, all supported control modes
   * (including MAXMotion), arb feedforward input, voltage compensation, limit switches, soft
   * limits, and current limiting, with algorithms translated directly from the Spark firmware.
   *
   * <p>This method will update the CANSparkSim's position and velocity, accessible with {@link
   * SparkSim#getPosition()} and {@link SparkSim#getVelocity()}. These values are automatically used
   * as the selected feedback sensor for calculations like closed-loop control and soft limits, and
   * are reflected in the selected sensor's value. Other sensors each have their own Sim class,
   * which can be used to inject their positions based on these calculations, to match how they are
   * configured physically. For example, to represent an Absolute Encoder on a 1:5 ratio from the
   * mechanism, {@link SparkAbsoluteEncoderSim#iterate(double, double)} is called each
   * simulationPeriodic loop with a velocity divided by 5. The selected sensor's position and
   * velocity will automatically be updated to match the CANSparkSim's when this method is called.
   *
   * @param velocity The externally calculated velocity in units after conversion. For example, if
   *     the velocity factor is 1, use RPM. If the velocity factor is (1 / 60) use RPS. The internal
   *     simulation state will 'lag' slightly behind this input due to the SPARK Device internal
   *     filtering. Noise will also be added.
   * @param vbus Bus voltage in volts (See WPILib's BatterySim class to simulate this, or use 12V)
   * @param dt Simulation time step in seconds
   */
  public void iterate(double velocity, double vbus, double dt) {
    if (vbus == 0) {
      DriverStation.reportError(
          "[REVLib Simulation error] "
              + m_deviceName
              + ": vbus provided to .iterate() cannot be zero",
          false);
      return;
    }

    // Velocity input is the system simulated input.
    double internalVelocity = NoiseGenerator.hallSensorVelocity(velocity);
    m_velocityAverage.put(internalVelocity, dt);
    internalVelocity = m_velocityAverage.get();

    // First set the states that are given
    m_velocity.set(internalVelocity);

    double positionFactor = 0;
    double velocityFactor = 0;
    if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
      if (((SparkFlex) m_spark).configAccessor.closedLoop.getFeedbackSensor()
          == FeedbackSensor.kAbsoluteEncoder) {
        positionFactor =
            ((SparkFlex) m_spark).configAccessor.absoluteEncoder.getPositionConversionFactor();
        velocityFactor =
            ((SparkFlex) m_spark).configAccessor.absoluteEncoder.getVelocityConversionFactor();
      } else {
        positionFactor = ((SparkFlex) m_spark).configAccessor.encoder.getPositionConversionFactor();
        velocityFactor = ((SparkFlex) m_spark).configAccessor.encoder.getVelocityConversionFactor();
      }
    } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
      if (((SparkMax) m_spark).configAccessor.closedLoop.getFeedbackSensor()
          == FeedbackSensor.kAbsoluteEncoder) {
        positionFactor =
            ((SparkMax) m_spark).configAccessor.absoluteEncoder.getPositionConversionFactor();
        velocityFactor =
            ((SparkMax) m_spark).configAccessor.absoluteEncoder.getVelocityConversionFactor();
      } else {
        positionFactor = ((SparkMax) m_spark).configAccessor.encoder.getPositionConversionFactor();
        velocityFactor = ((SparkMax) m_spark).configAccessor.encoder.getVelocityConversionFactor();
      }
    }

    // In the 2022 beta the above were not being set correctly. Seems to work now...
    // Either way for now this will prevent divide by 0 errors.
    if (positionFactor == 0.0) {
      positionFactor = 1.0;
    }
    if (velocityFactor == 0.0) {
      velocityFactor = 1.0;
    }

    double velocityRPM = velocity / velocityFactor;
    m_position.set(m_position.get() + ((velocityRPM / 60) * dt) * positionFactor);
    m_busVoltage.set(vbus);

    // Calculate the applied output
    double appliedOutput = 0.0;
    switch (m_controlMode.get()) {
        // Duty Cycle
      case 0:
        appliedOutput = m_setpoint.get();
        break;

        // Velocity
      case 1:
        appliedOutput =
            CANSparkJNI.c_Spark_GetSimClosedLoopOutput(
                m_spark.sparkHandle,
                (float) m_setpoint.get(),
                (float) internalVelocity,
                (float) dt);
        break;

        // Voltage
      case 2:
        appliedOutput = m_setpoint.get() / vbus;
        break;

        // Position
      case 3:
        appliedOutput =
            CANSparkJNI.c_Spark_GetSimClosedLoopOutput(
                m_spark.sparkHandle,
                (float) m_setpoint.get(),
                (float) m_position.get(),
                (float) dt);
        break;

        // Current
      case 4:
        appliedOutput =
            CANSparkJNI.c_Spark_GetSimClosedLoopOutput(
                m_spark.sparkHandle,
                (float) m_setpoint.get(),
                (float) m_motorCurrent.get(),
                (float) dt);
        break;

        // MAXMotion Position Control
      case 5:
        appliedOutput =
            CANSparkJNI.c_Spark_GetSimMAXMotionPositionControlOutput(
                m_spark.sparkHandle, (float) dt);
        break;

        // MAXMotion Position Control
      case 6:
        appliedOutput =
            CANSparkJNI.c_Spark_GetSimMAXMotionVelocityControlOutput(
                m_spark.sparkHandle, (float) dt);
        break;

      default:
        DriverStation.reportError(
            "[REVLib Simulation error] " + m_deviceName + ": Control mode out of bounds", false);
        break;
    }

    // ArbFF
    if (m_arbFFUnits.get() == 0) {
      // Voltage
      appliedOutput += m_arbFF.get() / vbus;
    } else {
      // Duty Cycle
      appliedOutput += m_arbFF.get();
    }

    // Voltage Compensation
    // exclude voltage closed loop mode and disabled mode
    int VOLTAGE_COMP_MODE_ID = 74;
    if (CANSparkJNI.c_Spark_GetParameterUint32(m_spark.sparkHandle, VOLTAGE_COMP_MODE_ID) == 2) {
      if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
        appliedOutput =
            (appliedOutput * ((SparkFlex) m_spark).configAccessor.getVoltageCompensation()) / vbus;
      } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
        appliedOutput =
            (appliedOutput * ((SparkMax) m_spark).configAccessor.getVoltageCompensation()) / vbus;
      }
    }

    // Limit to [-1, 1] or limit switch value
    double maxOutput = runLimitLogic(true) ? 0 : 1;
    double minOutput = runLimitLogic(false) ? 0 : -1;
    appliedOutput = Math.min(Math.max(appliedOutput, minOutput), maxOutput);

    appliedOutput =
        CANSparkJNI.c_Spark_GetSimCurrentLimitOutput(
            m_spark.sparkHandle, (float) appliedOutput, (float) m_motorCurrent.get());
    m_motorCurrent.set(m_dcMotor.getCurrent(Math.PI * velocityRPM / 30, appliedOutput * vbus));

    // check for faults
    SparkBase.Faults motorFaults = m_spark.getFaults();
    SparkBase.Faults motorStickyFaults = m_spark.getStickyFaults();
    if (motorFaults.can
        || motorStickyFaults.can
        || motorFaults.escEeprom
        || motorStickyFaults.escEeprom
        || motorFaults.motorType
        || motorStickyFaults.motorType
        || motorFaults.firmware
        || motorStickyFaults.firmware
        || motorFaults.gateDriver
        || motorStickyFaults.gateDriver
        || motorFaults.sensor
        || motorStickyFaults.sensor
        || motorFaults.temperature
        || motorStickyFaults.temperature
        || motorFaults.other
        || motorStickyFaults.other) {
      appliedOutput = 0;
      DriverStation.reportWarning(
          "[REVLib Simulation] " + m_deviceName + ": Device stopped due to fault", false);
    }

    // And finally, set remaining states
    boolean doEnable = false;
    if (m_enable == null) {
      doEnable = DriverStation.isEnabled();
    } else {
      doEnable = m_enable;
    }

    if (doEnable) {
      m_appliedOutput.set(appliedOutput);
    } else {
      m_appliedOutput.set(0.0);
      m_motorCurrent.set(0.0);
    }

    // mirror position/velocity to selected sensor
    FeedbackSensor selectedFeedbackSensor;
    if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
      selectedFeedbackSensor = ((SparkFlex) m_spark).configAccessor.closedLoop.getFeedbackSensor();
    } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
      selectedFeedbackSensor = ((SparkMax) m_spark).configAccessor.closedLoop.getFeedbackSensor();
    } else {
      selectedFeedbackSensor =
          FeedbackSensor.kNoSensor; // if unknown device, quietly skip selected sensor mirroring
    }

    switch (selectedFeedbackSensor) {
      case kPrimaryEncoder:
        SparkRelativeEncoderSim relativeEncoderSim = getRelativeEncoderSim();
        relativeEncoderSim.setPosition(m_position.get());
        relativeEncoderSim.setVelocity(m_velocity.get());
        break;
      case kAnalogSensor:
        SparkAnalogSensorSim analogSensorSim = getAnalogSensorSim();
        analogSensorSim.setPosition(m_position.get());
        analogSensorSim.setVelocity(m_velocity.get());
        break;
      case kAlternateOrExternalEncoder:
        if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
          SparkFlexExternalEncoderSim externalEncoderSim =
              new SparkFlexSim((SparkFlex) m_spark, m_dcMotor).getExternalEncoderSim();
          externalEncoderSim.setPosition(m_position.get());
          externalEncoderSim.setVelocity(m_velocity.get());
        } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
          SparkMaxAlternateEncoderSim alternateEncoderSim =
              new SparkMaxSim((SparkMax) m_spark, m_dcMotor).getAlternateEncoderSim();
          alternateEncoderSim.setPosition(m_position.get());
          alternateEncoderSim.setVelocity(m_velocity.get());
        }
        break;
      case kAbsoluteEncoder:
        SparkAbsoluteEncoderSim absoluteEncoderSim = getAbsoluteEncoderSim();
        absoluteEncoderSim.setPosition(m_position.get());
        absoluteEncoderSim.setVelocity(m_velocity.get());
        break;
      default:
        // do nothing if no sensor
    }
  }

  /**
   * Get the simulation velocity. This should be equivalent to calling CANEncoder().getVelocity()
   *
   * @return Velocity of the SPARK Device accounting for conversion factor
   */
  public double getVelocity() {
    return m_velocity.get();
  }

  /**
   * Set the simulation velocity. This method expects units after the conversion factor (your
   * program's native units).
   *
   * <p>Only use this method if not calling iterate()
   *
   * @param velocity simulation velocity
   */
  public void setVelocity(double velocity) {
    m_velocity.set(velocity);
  }

  /**
   * Get the simulation position. This should be equivalent to calling CANEncoder().getPosition()
   *
   * @return Velocity of the SPARK Device
   */
  public double getPosition() {
    return m_position.get();
  }

  /**
   * Set the simulated position. This is equivalent to calling CANEncoder().setPosition(), in fact
   * you probably are using that unless you have a good reason to set the sim value separately, or
   * are running simulation without using iterate()
   *
   * @param position simulated position in your programs units (after conversion)
   */
  public void setPosition(double position) {
    m_position.set(position);
  }

  /**
   * Get the simulated bus voltage
   *
   * @return simulated bus voltage in volts
   */
  public double getBusVoltage() {
    return m_busVoltage.get();
  }

  /**
   * Set the simulated bus voltage. Use this if you are not using the iterate() method.
   *
   * @param voltage bus voltage in volts
   */
  public void setBusVoltage(double voltage) {
    m_busVoltage.set(voltage);
  }

  /**
   * Get the simulated motor current in amps. This is equivalent to running spark.getOutputCurrent()
   *
   * @return motor current in amps
   */
  public double getMotorCurrent() {
    return m_motorCurrent.get();
  }

  /**
   * Set the simulated motor current. The iterate() method also sets this value. If you are using an
   * external method to calculate the current, but still want to use the iterate() method, call this
   * function *after* iterate()
   *
   * @param current current in amps
   */
  public void setMotorCurrent(double current) {
    m_motorCurrent.set(current);
  }

  /** Enable the Spark Device, allowing the motor to run. */
  public void enable() {
    m_enable = true;
  }

  /** Disable the Spark Device, causing the output to go to 0 */
  public void disable() {
    m_enable = false;
  }

  /**
   * Use the driver station enable as the method to enable/disable the Spark Max. This is the
   * default, so you do not need to call this unless you previously called enable() or disable().
   */
  public void useDriverStationEnable() {
    m_enable = null;
  }

  /**
   * Get the {@link SparkRelativeEncoderSim} object associated with this Spark Device. This will
   * allow you to read/write data from the simulated sensor and view it in the Sim GUI.
   *
   * @return The {@link SparkRelativeEncoderSim} object associated with this Spark Device
   */
  public SparkRelativeEncoderSim getRelativeEncoderSim() {
    // let the driver check if sim is running and create sim if necessary
    CANSparkJNI.c_Spark_CreateRelativeEncoderSim(m_spark.sparkHandle);

    if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
      return new SparkRelativeEncoderSim((SparkMax) m_spark);
    } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
      return new SparkRelativeEncoderSim((SparkFlex) m_spark);
    } else {
      DriverStation.reportError(
          "[REVLib Simulation error] " + m_deviceName + ": Unknown Device Type", false);
      return null;
    }
  }

  /**
   * Get the {@link SparkAbsoluteEncoderSim} object associated with this Spark Device. This will
   * allow you to read/write data from the simulated sensor and view it in the Sim GUI.
   *
   * @return The {@link SparkAbsoluteEncoderSim} object associated with this Spark Device
   */
  public SparkAbsoluteEncoderSim getAbsoluteEncoderSim() {
    // let the driver check if sim is running and create sim if necessary
    CANSparkJNI.c_Spark_CreateAbsoluteEncoderSim(m_spark.sparkHandle);

    if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
      return new SparkAbsoluteEncoderSim((SparkMax) m_spark);
    } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
      return new SparkAbsoluteEncoderSim((SparkFlex) m_spark);
    } else {
      DriverStation.reportError(
          "[REVLib Simulation error] " + m_deviceName + ": Unknown Device Type", false);
      return null;
    }
  }

  /**
   * Get the {@link SparkAnalogSensorSim} object associated with this Spark Device. This will allow
   * you to read/write data from the simulated sensor and view it in the Sim GUI.
   *
   * @return The {@link SparkAnalogSensorSim} object associated with this Spark Device
   */
  public SparkAnalogSensorSim getAnalogSensorSim() {
    // let the driver check if sim is running and create sim if necessary
    CANSparkJNI.c_Spark_CreateAnalogSensorSim(m_spark.sparkHandle);

    if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
      return new SparkAnalogSensorSim((SparkMax) m_spark);
    } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
      return new SparkAnalogSensorSim((SparkFlex) m_spark);
    } else {
      DriverStation.reportError(
          "[REVLib Simulation error] " + m_deviceName + ": Unknown Device Type", false);
      return null;
    }
  }

  /**
   * Get the Forward {@link SparkLimitSwitchSim} object associated with this Spark Device. This will
   * allow you to read/write data from the simulated sensor and view it in the Sim GUI.
   *
   * @return The Forward {@link SparkLimitSwitchSim} object associated with this Spark Device
   */
  public SparkLimitSwitchSim getForwardLimitSwitchSim() {
    if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
      return new SparkLimitSwitchSim((SparkMax) m_spark, true);
    } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
      return new SparkLimitSwitchSim((SparkFlex) m_spark, true);
    } else {
      DriverStation.reportError(
          "[REVLib Simulation error] " + m_deviceName + ": Unknown Device Type", false);
      return null;
    }
  }

  /**
   * Get the Reverse {@link SparkLimitSwitchSim} object associated with this Spark Device. This will
   * allow you to read/write data from the simulated sensor and view it in the Sim GUI.
   *
   * @return The Reverse {@link SparkLimitSwitchSim} object associated with this Spark Device
   */
  public SparkLimitSwitchSim getReverseLimitSwitchSim() {
    if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
      return new SparkLimitSwitchSim((SparkMax) m_spark, false);
    } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
      return new SparkLimitSwitchSim((SparkFlex) m_spark, false);
    } else {
      DriverStation.reportError(
          "[REVLib Simulation error] " + m_deviceName + ": Unknown Device Type", false);
      return null;
    }
  }

  /**
   * Get the {@link SparkSimFaultManager} object associated with this Spark Device. This will allow
   * you to set simulated faults on your simulated device and view the Fault Manager in the Sim GUI.
   *
   * @return The {@link SparkSimFaultManager} object associated with this Spark Device
   */
  public SparkSimFaultManager getFaultManager() {
    if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkMax) {
      return new SparkSimFaultManager((SparkMax) m_spark);
    } else if (m_spark.getSparkModel() == SparkLowLevel.SparkModel.SparkFlex) {
      return new SparkSimFaultManager((SparkFlex) m_spark);
    } else {
      DriverStation.reportError(
          "[REVLib Simulation error] " + m_deviceName + ": Unknown Device Type", false);
      return null;
    }
  }
}
