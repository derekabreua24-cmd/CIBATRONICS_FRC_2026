/*
 * Copyright (c) 2024-2026 REV Robotics
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

/*
 * This file is auto-generated. Do NOT modify it directly.
 * See https://github.com/REVrobotics/SparkParameters
 */

package com.revrobotics.spark.config;

public enum SparkParameters {
  kCANID(0, Type.UINT32),
  kInputMode(1, Type.UINT32),
  kMotorType(2, Type.UINT32),
  kCommutationAdvance(3, Type.FLOAT),
  kControlType(5, Type.UINT32),
  kIdleMode(6, Type.UINT32),
  kInputDeadband(7, Type.FLOAT),
  kClosedLoopControlSensor(9, Type.UINT32),
  kPolePairs(10, Type.UINT32),
  kCurrentChop(11, Type.FLOAT),
  kCurrentChopCycles(12, Type.UINT32),
  kP_0(13, Type.FLOAT),
  kI_0(14, Type.FLOAT),
  kD_0(15, Type.FLOAT),
  kV_0(16, Type.FLOAT),
  kIZone_0(17, Type.FLOAT),
  kDFilter_0(18, Type.FLOAT),
  kOutputMin_0(19, Type.FLOAT),
  kOutputMax_0(20, Type.FLOAT),
  kP_1(21, Type.FLOAT),
  kI_1(22, Type.FLOAT),
  kD_1(23, Type.FLOAT),
  kV_1(24, Type.FLOAT),
  kIZone_1(25, Type.FLOAT),
  kDFilter_1(26, Type.FLOAT),
  kOutputMin_1(27, Type.FLOAT),
  kOutputMax_1(28, Type.FLOAT),
  kP_2(29, Type.FLOAT),
  kI_2(30, Type.FLOAT),
  kD_2(31, Type.FLOAT),
  kV_2(32, Type.FLOAT),
  kIZone_2(33, Type.FLOAT),
  kDFilter_2(34, Type.FLOAT),
  kOutputMin_2(35, Type.FLOAT),
  kOutputMax_2(36, Type.FLOAT),
  kP_3(37, Type.FLOAT),
  kI_3(38, Type.FLOAT),
  kD_3(39, Type.FLOAT),
  kV_3(40, Type.FLOAT),
  kIZone_3(41, Type.FLOAT),
  kDFilter_3(42, Type.FLOAT),
  kOutputMin_3(43, Type.FLOAT),
  kOutputMax_3(44, Type.FLOAT),
  kInverted(45, Type.BOOL),
  kLimitSwitchFwdPolarity(50, Type.BOOL),
  kLimitSwitchRevPolarity(51, Type.BOOL),
  kHardLimitFwdEn(52, Type.UINT32),
  kHardLimitRevEn(53, Type.UINT32),
  kSoftLimitFwdEn(54, Type.BOOL),
  kSoftLimitRevEn(55, Type.BOOL),
  kOpenLoopRampRate(56, Type.FLOAT),
  kLegacyFollowerID(57, Type.UINT32),
  kLegacyFollowerConfig(58, Type.UINT32),
  kSmartCurrentStallLimit(59, Type.UINT32),
  kSmartCurrentFreeLimit(60, Type.UINT32),
  kSmartCurrentConfig(61, Type.UINT32),
  kSmartCurrentReserved(62, Type.UINT32),
  kMotorKv(63, Type.UINT32),
  kEncoderCountsPerRev(69, Type.UINT32),
  kEncoderAverageDepth(70, Type.UINT32),
  kEncoderSampleDelta(71, Type.UINT32),
  kEncoderInverted(72, Type.BOOL),
  kVoltageCompensationMode(74, Type.UINT32),
  kCompensatedNominalVoltage(75, Type.FLOAT),
  kIMaxAccum_0(96, Type.FLOAT),
  kAllowedClosedLoopError_0(97, Type.FLOAT),
  kIMaxAccum_1(100, Type.FLOAT),
  kAllowedClosedLoopError_1(101, Type.FLOAT),
  kIMaxAccum_2(104, Type.FLOAT),
  kAllowedClosedLoopError_2(105, Type.FLOAT),
  kIMaxAccum_3(108, Type.FLOAT),
  kAllowedClosedLoopError_3(109, Type.FLOAT),
  kPositionConversionFactor(112, Type.FLOAT),
  kVelocityConversionFactor(113, Type.FLOAT),
  kClosedLoopRampRate(114, Type.FLOAT),
  kSoftLimitForward(115, Type.FLOAT),
  kSoftLimitReverse(116, Type.FLOAT),
  kAnalogPositionConversion(119, Type.FLOAT),
  kAnalogVelocityConversion(120, Type.FLOAT),
  kAnalogAverageDepth(121, Type.UINT32),
  kAnalogSensorMode(122, Type.UINT32),
  kAnalogInverted(123, Type.BOOL),
  kAnalogSampleDelta(124, Type.UINT32),
  kCompatibilityPortConfig(127, Type.UINT32),
  kAltEncoderCountsPerRev(128, Type.UINT32),
  kAltEncoderAverageDepth(129, Type.UINT32),
  kAltEncoderSampleDelta(130, Type.UINT32),
  kAltEncoderInverted(131, Type.BOOL),
  kAltEncoderPositionConversion(132, Type.FLOAT),
  kAltEncoderVelocityConversion(133, Type.FLOAT),
  kUvwSensorSampleRate(136, Type.FLOAT),
  kUvwSensorAverageDepth(137, Type.UINT32),
  kNumParameters(138, Type.UINT32),
  kDutyCyclePositionFactor(139, Type.FLOAT),
  kDutyCycleVelocityFactor(140, Type.FLOAT),
  kDutyCycleInverted(141, Type.BOOL),
  kDutyCycleSensorMode(142, Type.UINT32),
  kDutyCycleAverageDepth(143, Type.UINT32),
  kDutyCycleOffsetLegacy(145, Type.FLOAT),
  kPositionPIDWrapEnable(149, Type.BOOL),
  kPositionPIDMinInput(150, Type.FLOAT),
  kPositionPIDMaxInput(151, Type.FLOAT),
  kDutyCycleZeroCentered(152, Type.BOOL),
  kDutyCycleSensorPrescaler(153, Type.UINT32),
  kDutyCycleOffset(154, Type.FLOAT),
  kProductId(155, Type.UINT32),
  kDeviceMajorVersion(156, Type.UINT32),
  kDeviceMinorVersion(157, Type.UINT32),
  kStatus0Period(158, Type.UINT32),
  kStatus1Period(159, Type.UINT32),
  kStatus2Period(160, Type.UINT32),
  kStatus3Period(161, Type.UINT32),
  kStatus4Period(162, Type.UINT32),
  kStatus5Period(163, Type.UINT32),
  kStatus6Period(164, Type.UINT32),
  kStatus7Period(165, Type.UINT32),
  kMAXMotionCruiseVelocity_0(166, Type.FLOAT),
  kMAXMotionMaxAccel_0(167, Type.FLOAT),
  kMAXMotionMaxJerk_0(168, Type.FLOAT),
  kMAXMotionAllowedProfileError_0(169, Type.FLOAT),
  kMAXMotionPositionMode_0(170, Type.UINT32),
  kMAXMotionCruiseVelocity_1(171, Type.FLOAT),
  kMAXMotionMaxAccel_1(172, Type.FLOAT),
  kMAXMotionMaxJerk_1(173, Type.FLOAT),
  kMAXMotionAllowedProfileError_1(174, Type.FLOAT),
  kMAXMotionPositionMode_1(175, Type.UINT32),
  kMAXMotionCruiseVelocity_2(176, Type.FLOAT),
  kMAXMotionMaxAccel_2(177, Type.FLOAT),
  kMAXMotionMaxJerk_2(178, Type.FLOAT),
  kMAXMotionAllowedProfileError_2(179, Type.FLOAT),
  kMAXMotionPositionMode_2(180, Type.UINT32),
  kMAXMotionCruiseVelocity_3(181, Type.FLOAT),
  kMAXMotionMaxAccel_3(182, Type.FLOAT),
  kMAXMotionMaxJerk_3(183, Type.FLOAT),
  kMAXMotionAllowedProfileError_3(184, Type.FLOAT),
  kMAXMotionPositionMode_3(185, Type.UINT32),
  kForceEnableStatus_0(186, Type.BOOL),
  kForceEnableStatus_1(187, Type.BOOL),
  kForceEnableStatus_2(188, Type.BOOL),
  kForceEnableStatus_3(189, Type.BOOL),
  kForceEnableStatus_4(190, Type.BOOL),
  kForceEnableStatus_5(191, Type.BOOL),
  kForceEnableStatus_6(192, Type.BOOL),
  kForceEnableStatus_7(193, Type.BOOL),
  kFollowerModeLeaderId(194, Type.UINT32),
  kFollowerModeIsInverted(195, Type.BOOL),
  kDutyCycleEncoderStartPulseUs(196, Type.FLOAT),
  kDutyCycleEncoderEndPulseUs(197, Type.FLOAT),
  kParamTableVersion(198, Type.UINT32),
  kStatus8Period(199, Type.UINT32),
  kForceEnableStatus_8(200, Type.BOOL),
  kLimitSwitchPositionSensor(201, Type.UINT32),
  kLimitSwitchFwdPosition(202, Type.FLOAT),
  kLimitSwitchRevPosition(203, Type.FLOAT),
  kS_0(204, Type.FLOAT),
  kA_0(205, Type.FLOAT),
  kG_0(206, Type.FLOAT),
  kCos_0(207, Type.FLOAT),
  kCosRatio_0(208, Type.FLOAT),
  kS_1(209, Type.FLOAT),
  kA_1(210, Type.FLOAT),
  kG_1(211, Type.FLOAT),
  kCos_1(212, Type.FLOAT),
  kCosRatio_1(213, Type.FLOAT),
  kS_2(214, Type.FLOAT),
  kA_2(215, Type.FLOAT),
  kG_2(216, Type.FLOAT),
  kCos_2(217, Type.FLOAT),
  kCosRatio_2(218, Type.FLOAT),
  kS_3(219, Type.FLOAT),
  kA_3(220, Type.FLOAT),
  kG_3(221, Type.FLOAT),
  kCos_3(222, Type.FLOAT),
  kCosRatio_3(223, Type.FLOAT),
  kStatus9Period(224, Type.UINT32),
  kForceEnableStatus_9(225, Type.BOOL),
  kDetachedEncoderDeviceID(226, Type.UINT32);

  @SuppressWarnings("MemberName")
  public final int value;

  public final Type type;

  SparkParameters(int value, Type type) {
    this.value = value;
    this.type = type;
  }

  public enum Type {
    UINT32,
    INT32,
    FLOAT,
    BOOL
  }
}
