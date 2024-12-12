#include "battery_monitor.h"

volatile bool BatteryMonitor::alertFlag = false;

BatteryMonitor::BatteryMonitor() 
    : batteryPercentage(0.0f)
    , batteryVoltage(0.0f)
    , alertTriggered(false) {
}

void IRAM_ATTR BatteryMonitor::alertISR() {
    alertFlag = true;
}

bool BatteryMonitor::init() {
    Wire.begin();
    
    // Initialize the MAX17043
    if (!fuelGauge.begin()) {
        Serial.println("Failed to initialize MAX17043!");
        return false;
    }
    
    // Configure alert pin as input with pull-up
    pinMode(ALERT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ALERT_PIN), alertISR, FALLING);
    
    // Set default alert threshold to 15%
    fuelGauge.setAlertThreshold(LOW_BATTERY_THRESHOLD);
    
    // Quick start the gauge
    fuelGauge.quickStart();
    
    return true;
}

void BatteryMonitor::update() {
    // Read battery status
    batteryPercentage = fuelGauge.getSOC();
    batteryVoltage = fuelGauge.getVoltage();
    
    // Check for alert condition
    if (alertFlag) {
        alertTriggered = true;
        alertFlag = false;
        
        // Log low battery warning
        Serial.println("Low battery alert triggered!");
        Serial.printf("Battery: %.1f%%, Voltage: %.2fV\n", 
                     batteryPercentage, batteryVoltage);
    }
}

float BatteryMonitor::getBatteryPercentage() const {
    return batteryPercentage;
}

float BatteryMonitor::getBatteryVoltage() const {
    return batteryVoltage;
}

bool BatteryMonitor::isLowBattery() const {
    return batteryPercentage <= LOW_BATTERY_THRESHOLD || alertTriggered;
}

void BatteryMonitor::setAlertThreshold(uint8_t percentage) {
    fuelGauge.setAlertThreshold(percentage);
}

void BatteryMonitor::clearAlert() {
    alertTriggered = false;
    fuelGauge.clearAlert();
} 