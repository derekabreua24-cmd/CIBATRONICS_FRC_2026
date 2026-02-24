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

package com.revrobotics.config;

import com.revrobotics.jni.CANCommonJNI;
import com.revrobotics.jni.CANServoHubJNI;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.jni.DetachedEncoderJNI;
import java.util.HashMap;
import java.util.Map;

public abstract class BaseConfig {
  private Map<Integer, Object> parameters = new HashMap<>();

  protected enum CANType {
    kSpark(0),
    kServoHub(1),
    kDetachedEncoder(2);

    @SuppressWarnings("MemberName")
    public final int value;

    CANType(int value) {
      this.value = value;
    }
  }

  private interface GetParameteraTypeFunctionPtr {
    int getParameterTypeFunction(int key);
  }

  private GetParameteraTypeFunctionPtr getParameterTypeFuncPtr;

  protected BaseConfig(CANType canType) {
    if (canType == CANType.kSpark) {
      getParameterTypeFuncPtr = CANSparkJNI::c_Spark_GetParameterType;
    } else if (canType == CANType.kServoHub) {
      getParameterTypeFuncPtr = CANServoHubJNI::c_ServoHub_GetParameterType;
    } else if (canType == CANType.kDetachedEncoder) {
      getParameterTypeFuncPtr = DetachedEncoderJNI::getParameterType;
    }
  }

  protected void putParameter(int parameterId, int value) {
    parameters.put(parameterId, value);
  }

  protected void putParameter(int parameterId, float value) {
    parameters.put(parameterId, value);
  }

  protected void putParameter(int parameterId, boolean value) {
    parameters.put(parameterId, value);
  }

  private void putParameter(int parameterId, Object value) {
    parameters.put(parameterId, value);
  }

  protected Object getParameter(int parameterId) {
    return parameters.get(parameterId);
  }

  protected Object getParameter(BaseConfig fromConfig, int parameterId) {
    return fromConfig.getParameter(parameterId);
  }

  protected void removeParameter(int parameterId) {
    parameters.remove(parameterId);
  }

  protected void removeParameter(BaseConfig fromConfig, int parameterId) {
    fromConfig.removeParameter(parameterId);
  }

  protected void apply(BaseConfig config) {
    for (Map.Entry<Integer, Object> parameter : config.parameters.entrySet()) {
      putParameter(parameter.getKey(), parameter.getValue());
    }
  }

  public String flatten() {
    String flattenedString = "";

    for (Map.Entry<Integer, Object> parameter : parameters.entrySet()) {
      switch (getParameterTypeFuncPtr.getParameterTypeFunction(parameter.getKey())) {
        case 1:
          flattenedString +=
              CANCommonJNI.c_REVLib_FlattenParameterInt32(
                  parameter.getKey(), (int) parameter.getValue());
          break;
        case 2:
          flattenedString +=
              CANCommonJNI.c_REVLib_FlattenParameterUint32(
                  parameter.getKey(), (int) parameter.getValue());
          break;
        case 3:
          flattenedString +=
              CANCommonJNI.c_REVLib_FlattenParameterFloat(
                  parameter.getKey(), (float) parameter.getValue());
          break;
        case 4:
          flattenedString +=
              CANCommonJNI.c_REVLib_FlattenParameterBool(
                  parameter.getKey(), (boolean) parameter.getValue());
          break;
      }
    }

    return flattenedString;
  }
}
