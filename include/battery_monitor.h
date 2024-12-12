#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Wire.h>
#include <MAX17043.h>

class BatteryMonitor {
public:
    BatteryMonitor();
    bool init();
    void update();
    
    // Getter methods
    float getBatteryPercentage() const;
    float getBatteryVoltage() const;
    bool isLowBattery() const {
        return batteryPercentage <= LOW_BATTERY_THRESHOLD || alertTriggered;
    }
    
    // Configuration methods
    void setAlertThreshold(uint8_t percentage);
    void clearAlert();

private:
    MAX17043 fuelGauge;
    static const uint8_t ALERT_PIN = 35;  // GPIO pin for alert interrupt
    static const float LOW_BATTERY_THRESHOLD = 15.0f;  // 15% threshold
    
    float batteryPercentage;
    float batteryVoltage;
    bool alertTriggered;
    
    static void IRAM_ATTR alertISR();
    static volatile bool alertFlag;
};

volatile bool BatteryMonitor::alertFlag = false;

#endif 