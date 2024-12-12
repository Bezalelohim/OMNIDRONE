#ifndef CONTROL_ALGORITHMS_H
#define CONTROL_ALGORITHMS_H

#include "communication.h"
#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    void setParameters(float kp, float ki, float kd);
    float compute(float setpoint, float measurement, float dt);
    void reset();

private:
    float kp, ki, kd;
    float lastError;
    float integral;
    float lastMeasurement;
};

class FlightController {
public:
    FlightController();
    void init();
    void updateControlLoop(const ControlInputs& inputs, 
                          float currentRoll, float currentPitch, float currentYaw,
                          float currentAltitude = 0.0f);
    void getMotorOutputs(float& m1, float& m2, float& m3, float& m4);
    void setAltitudeHoldPID(float kp, float ki, float kd);
    void reset() {
        rollPID.reset();
        pitchPID.reset();
        yawPID.reset();
        altitudePID.reset();
    }
    float getLoopTime() const { return loopTime; }
    bool isTimingHealthy() const { return loopTime < 0.004f; }

private:
    PIDController rollPID;
    PIDController pitchPID;
    PIDController yawPID;
    PIDController altitudePID;
    float motorOutputs[4];
    
    static constexpr float ROLL_KP = 1.0f;
    static constexpr float ROLL_KI = 0.1f;
    static constexpr float ROLL_KD = 0.05f;
    
    static constexpr float PITCH_KP = 1.0f;
    static constexpr float PITCH_KI = 0.1f;
    static constexpr float PITCH_KD = 0.05f;
    
    static constexpr float YAW_KP = 2.0f;
    static constexpr float YAW_KI = 0.15f;
    static constexpr float YAW_KD = 0.1f;
    
    static constexpr float ALTITUDE_KP = 0.8f;
    static constexpr float ALTITUDE_KI = 0.1f;
    static constexpr float ALTITUDE_KD = 0.2f;
    
    unsigned long lastUpdateTime;
    float loopTime;
};

#endif 