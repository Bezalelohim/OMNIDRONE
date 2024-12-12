#include <Arduino.h>
#include "sensor.h"
#include "motor_control.h"
#include "communication.h"
#include "control_algorithms.h"
#include "battery_monitor.h"
#include "lidar_sensor.h"

// Global objects
SensorSystem sensor;
MotorControl motors;
Communication comm;
FlightController controller;
BatteryMonitor battery;
LidarSensor lidar;

// Timing variables
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL = 4000; // 250Hz loop rate (in microseconds)

// Battery safety thresholds
const float CRITICAL_BATTERY_THRESHOLD = 10.0f;  // Critical level for emergency landing
const float RTH_BATTERY_THRESHOLD = 25.0f;       // Return to home threshold
const float SAFE_BATTERY_THRESHOLD = 30.0f;      // Warning threshold

// Flight states
enum FlightState {
    NORMAL_OPERATION,
    WARNING_LOW_BATTERY,
    RETURN_TO_HOME,
    EMERGENCY_LANDING,
    FAILSAFE_MODE,
    ALTITUDE_HOLD    // New state for altitude hold
};

FlightState currentState = NORMAL_OPERATION;

// Function declarations
void handleBatteryStatus();
void executeEmergencyLanding();I
void executeReturnToHome();
void updateTelemetry();
bool checkSystemHealth();
void handleStateTransition(FlightState newState);

// Add altitude control parameters
const float ALTITUDE_THRESHOLD = 0.05f;  // 5cm threshold for altitude hold
const float MAX_ALTITUDE = 5.0f;         // Maximum allowed altitude in meters
const float MIN_ALTITUDE = 0.3f;         // Minimum allowed altitude in meters

// Add at the top with other global variables
unsigned long lastStatusPrint = 0;
const unsigned long STATUS_PRINT_INTERVAL = 1000; // 1 second

void setup() {
    Serial.begin(115200);
    
    // Initialize all subsystems
    if (!sensor.init()) {
        Serial.println("Sensor initialization failed!");
        while (1) delay(100);
    }
    
    if (!motors.init()) {
        Serial.println("Motor initialization failed!");
        while (1) delay(100);
    }
    
    if (!comm.init()) {
        Serial.println("Communication initialization failed!");
        while (1) delay(100);
    }
    
    if (!battery.init()) {
        Serial.println("Battery monitor initialization failed!");
        while (1) delay(100);
    }
    
    if (!lidar.init()) {
        Serial.println("LIDAR initialization failed!");
        while (1) delay(100);
    }
    
    // Set battery alert thresholds
    battery.setAlertThreshold(SAFE_BATTERY_THRESHOLD);
    
    controller.init();
    Serial.println("All systems initialized successfully!");
}

void loop() {
    unsigned long currentTime = micros();
    static unsigned long lastTime = currentTime;
    
    // Protect against micros() overflow
    if (currentTime < lastTime) {
        lastTime = currentTime;
        return;
    }
    
    if (currentTime - lastLoopTime >= LOOP_INTERVAL) {
        lastLoopTime = currentTime;
        
        // Update sensor readings
        sensor.update();
        battery.update();
        lidar.update();
        
        // Handle battery status and state transitions
        handleBatteryStatus();
        
        // Get control inputs
        ControlInputs inputs;
        if (comm.receiveControlInputs(inputs) && checkSystemHealth()) {
            // Validate and constrain target altitude
            if (inputs.altitudeHoldMode) {
                inputs.targetAltitude = constrain(inputs.targetAltitude, MIN_ALTITUDE, MAX_ALTITUDE);
                handleStateTransition(ALTITUDE_HOLD);
            }
            
            switch (currentState) {
                case NORMAL_OPERATION:
                    // Normal flight control
                    controller.updateControlLoop(inputs, 
                                              sensor.getRoll(),
                                              sensor.getPitch(),
                                              sensor.getYaw());
                    break;
                    
                case ALTITUDE_HOLD:
                    if (lidar.isValidReading()) {
                        // Use PID controller for altitude hold
                        controller.updateControlLoop(inputs, 
                                                  sensor.getRoll(),
                                                  sensor.getPitch(),
                                                  sensor.getYaw(),
                                                  lidar.getAltitude());
                    } else {
                        // Fall back to manual control if LIDAR reading is invalid
                        handleStateTransition(NORMAL_OPERATION);
                        Serial.println("LIDAR reading invalid, disabling altitude hold");
                    }
                    break;
                    
                case WARNING_LOW_BATTERY:
                    // Limit maximum throttle to conserve power
                    inputs.throttle *= 0.8f;
                    controller.updateControlLoop(inputs, 
                                              sensor.getRoll(),
                                              sensor.getPitch(),
                                              sensor.getYaw());
                    break;
                    
                case RETURN_TO_HOME:
                    executeReturnToHome();
                    break;
                    
                case EMERGENCY_LANDING:
                    executeEmergencyLanding();
                    break;
                    
                case FAILSAFE_MODE:
                    motors.emergencyStop();
                    break;
            }
            
            // Apply motor outputs if not in failsafe
            if (currentState != FAILSAFE_MODE) {
                float m1, m2, m3, m4;
                controller.getMotorOutputs(m1, m2, m3, m4);
                motors.updateMotors(m1, m2, m3, m4);
            }
            
            // Update and send telemetry
            updateTelemetry();
            
        } else {
            handleStateTransition(FAILSAFE_MODE);
            motors.emergencyStop();
            Serial.println("System health check failed - entering failsafe mode");
        }
        
        printSystemStatus();
    }
}

void handleBatteryStatus() {
    float batteryLevel = battery.getBatteryPercentage();
    
    // State machine for battery management
    switch (currentState) {
        case NORMAL_OPERATION:
            if (batteryLevel <= RTH_BATTERY_THRESHOLD) {
                handleStateTransition(RETURN_TO_HOME);
                Serial.println("Battery low! Initiating return to home...");
            } else if (batteryLevel <= SAFE_BATTERY_THRESHOLD) {
                handleStateTransition(WARNING_LOW_BATTERY);
                Serial.println("Battery warning! Limiting performance...");
            }
            break;
            
        case WARNING_LOW_BATTERY:
        case RETURN_TO_HOME:
            if (batteryLevel <= CRITICAL_BATTERY_THRESHOLD) {
                handleStateTransition(EMERGENCY_LANDING);
                Serial.println("CRITICAL BATTERY! Initiating emergency landing!");
            }
            break;
            
        case EMERGENCY_LANDING:
            // Continue emergency landing procedure
            break;
            
        case FAILSAFE_MODE:
            // Maintain failsafe state
            break;
    }
}

void executeEmergencyLanding() {
    // Implement gradual descent while maintaining stability
    ControlInputs landingInputs = {
        .throttle = 0.3f,  // Reduced power for controlled descent
        .roll = 0.0f,
        .pitch = 0.0f,
        .yaw = 0.0f
    };
    
    controller.updateControlLoop(landingInputs,
                               sensor.getRoll(),
                               sensor.getPitch(),
                               sensor.getYaw());
}

void executeReturnToHome() {
    // TODO: Implement return to home logic
    // This would require GPS integration and navigation algorithms
    // For now, just maintain stable hover
    ControlInputs hoverInputs = {
        .throttle = 0.5f,
        .roll = 0.0f,
        .pitch = 0.0f,
        .yaw = 0.0f
    };
    
    controller.updateControlLoop(hoverInputs,
                               sensor.getRoll(),
                               sensor.getPitch(),
                               sensor.getYaw());
}

void updateTelemetry() {
    TelemetryData telemetry = {
        .batteryVoltage = battery.getBatteryVoltage(),
        .batteryPercentage = battery.getBatteryPercentage(),
        .roll = sensor.getRoll(),
        .pitch = sensor.getPitch(),
        .yaw = sensor.getYaw(),
        .currentAltitude = lidar.getAltitude(),
        .altitudeHoldActive = (currentState == ALTITUDE_HOLD),
        .signalStrength = 100,
        .lowBatteryAlert = battery.isLowBattery()
    };
    
    comm.sendTelemetry(telemetry);
}

void printSystemStatus() {
    if (millis() - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
        lastStatusPrint = millis();
        
        Serial.println("\n=== System Status ===");
        Serial.printf("State: %d\n", currentState);
        Serial.printf("Battery: %.1f%% (%.2fV)\n", 
                     battery.getBatteryPercentage(),
                     battery.getBatteryVoltage());
        Serial.printf("Altitude: %.2fm\n", lidar.getAltitude());
        Serial.printf("Attitude - Roll: %.1f, Pitch: %.1f, Yaw: %.1f\n",
                     sensor.getRoll(), sensor.getPitch(), sensor.getYaw());
        Serial.printf("Radio: %s\n", 
                     comm.isConnectionHealthy() ? "Connected" : "Disconnected");
        Serial.println("==================\n");
    }
}

bool checkSystemHealth() {
    static unsigned long lastCheck = 0;
    const unsigned long CHECK_INTERVAL = 100; // 100ms interval
    
    if (millis() - lastCheck >= CHECK_INTERVAL) {
        lastCheck = millis();
        
        // Check all critical systems
        if (!sensor.isHealthy() || !lidar.isValidReading() || 
            !comm.isConnectionHealthy() || battery.isLowBattery()) {
            
            return false;
        }
    }
    return true;
}

void handleStateTransition(FlightState newState) {
    if (currentState != newState) {
        Serial.printf("State transition: %d -> %d\n", currentState, newState);
        
        // Handle specific transition actions
        switch (newState) {
            case ALTITUDE_HOLD:
                controller.reset();  // Reset PID controllers
                break;
                
            case EMERGENCY_LANDING:
                // Gradually reduce throttle
                break;
                
            case FAILSAFE_MODE:
                motors.emergencyStop();
                break;
        }
        
        currentState = newState;
    }
} 