#include "control_algorithms.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), lastError(0.0f), integral(0.0f), lastMeasurement(0.0f) {
}

void PIDController::setParameters(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    reset();
}

float PIDController::compute(float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // Proportional term
    float P = kp * error;
    
    // Integral term with anti-windup
    integral += ki * error * dt;
    integral = constrain(integral, -0.5f, 0.5f);
    float I = integral;
    
    // Derivative term (on measurement to avoid derivative kick)
    float dMeasurement = (measurement - lastMeasurement) / dt;
    float D = -kd * dMeasurement;
    
    lastError = error;
    lastMeasurement = measurement;
    
    return P + I + D;
}

void PIDController::reset() {
    lastError = 0.0f;
    integral = 0.0f;
    lastMeasurement = 0.0f;
}

FlightController::FlightController()
    : rollPID(ROLL_KP, ROLL_KI, ROLL_KD)
    , pitchPID(PITCH_KP, PITCH_KI, PITCH_KD)
    , yawPID(YAW_KP, YAW_KI, YAW_KD)
    , altitudePID(ALTITUDE_KP, ALTITUDE_KI, ALTITUDE_KD) {
    
    for (int i = 0; i < 4; i++) {
        motorOutputs[i] = 0.0f;
    }
}

void FlightController::init() {
    rollPID.reset();
    pitchPID.reset();
    yawPID.reset();
    altitudePID.reset();
}

void FlightController::updateControlLoop(const ControlInputs& inputs, 
                                       float currentRoll, float currentPitch, float currentYaw,
                                       float currentAltitude) {
    float dt = 0.004f;  // 250Hz loop rate = 4ms
    
    // Calculate PID outputs
    float rollOutput = rollPID.compute(inputs.roll, currentRoll, dt);
    float pitchOutput = pitchPID.compute(inputs.pitch, currentPitch, dt);
    float yawOutput = yawPID.compute(inputs.yaw, currentYaw, dt);
    
    // Calculate altitude control if in altitude hold mode
    float throttleOutput = inputs.throttle;
    if (inputs.altitudeHoldMode) {
        float altitudeOutput = altitudePID.compute(inputs.targetAltitude, currentAltitude, dt);
        throttleOutput = 0.5f + altitudeOutput;  // Base throttle + PID correction
        throttleOutput = constrain(throttleOutput, 0.1f, 0.9f);
    }
    
    // Mix outputs for quadcopter configuration (X configuration)
    // Motor order: FR, FL, RL, RR
    motorOutputs[0] = throttleOutput + rollOutput + pitchOutput + yawOutput;  // Front Right
    motorOutputs[1] = throttleOutput - rollOutput + pitchOutput - yawOutput;  // Front Left
    motorOutputs[2] = throttleOutput - rollOutput - pitchOutput + yawOutput;  // Rear Left
    motorOutputs[3] = throttleOutput + rollOutput - pitchOutput - yawOutput;  // Rear Right
    
    // Normalize outputs if any exceed 1.0
    float maxOutput = 0.0f;
    for (int i = 0; i < 4; i++) {
        maxOutput = max(maxOutput, abs(motorOutputs[i]));
    }
    
    if (maxOutput > 1.0f) {
        for (int i = 0; i < 4; i++) {
            motorOutputs[i] /= maxOutput;
        }
    }
    
    // Ensure minimum motor output for stability
    const float MIN_THROTTLE = 0.1f;
    if (throttleOutput > 0.1f) {  // Only apply when throttle is above minimum
        for (int i = 0; i < 4; i++) {
            motorOutputs[i] = max(motorOutputs[i], MIN_THROTTLE);
        }
    }
}

void FlightController::getMotorOutputs(float& m1, float& m2, float& m3, float& m4) {
    m1 = motorOutputs[0];
    m2 = motorOutputs[1];
    m3 = motorOutputs[2];
    m4 = motorOutputs[3];
}

void FlightController::setAltitudeHoldPID(float kp, float ki, float kd) {
    altitudePID.setParameters(kp, ki, kd);
} 