#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <RF24.h>
#include <SPI.h>

struct ControlInputs {
    float throttle;
    float roll;
    float pitch;
    float yaw;
    float targetAltitude;    // Desired altitude in meters
    bool altitudeHoldMode;   // Enable/disable altitude hold
};

struct TelemetryData {
    float batteryVoltage;
    float batteryPercentage;
    float roll;
    float pitch;
    float yaw;
    float currentAltitude;   // Current altitude from LIDAR
    bool altitudeHoldActive; // Indicates if altitude hold is active
    uint8_t signalStrength;
    bool lowBatteryAlert;
};

class Communication {
public:
    Communication();
    bool init();
    bool receiveControlInputs(ControlInputs& inputs);
    bool sendTelemetry(const TelemetryData& telemetry);
    bool isConnectionHealthy() const;

private:
    RF24 radio;
    static const uint8_t CE_PIN = 4;
    static const uint8_t CSN_PIN = 5;
    uint8_t address[2][6] = {"1Node", "2Node"};
};

#endif 