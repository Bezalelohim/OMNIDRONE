#include "lidar_sensor.h"

LidarSensor::LidarSensor()
    : currentAltitude(0.0f)
    , signalStrength(0)
    , noiseLevel(0)
    , validReading(false)
    , streamModeEnabled(false)
    , readIndex(0)
    , total(0.0f) {
    
    for (int i = 0; i < FILTER_SIZE; i++) {
        readings[i] = 0.0f;
    }
}

bool LidarSensor::init() {
    Wire.begin();
    Wire.setClock(400000);  // LW20 supports up to 400kHz I2C
    
    // Check if sensor is responding
    uint8_t productName[16];
    if (!readRegister(REG_PRODUCT_NAME, productName, 16)) {
        Serial.println("Failed to communicate with LW20!");
        return false;
    }
    
    // Configure for optimal drone operation
    setUpdateRate(200);     // 200Hz update rate - good balance of speed and stability
    setResolution(1);       // 1cm resolution for maximum precision
    setStreamMode(true);    // Enable continuous streaming mode
    
    delay(100);  // Allow sensor to settle
    
    return true;
}

void LidarSensor::update() {
    if (!streamModeEnabled) {
        // Trigger a single reading if not in stream mode
        uint8_t trigger = 1;
        writeRegister(REG_DISTANCE_OUTPUT, &trigger, 1);
        delay(1);  // Wait for measurement
    }
    
    // Read distance data (32-bit float)
    uint8_t distanceBytes[4];
    if (!readRegister(REG_DISTANCE_OUTPUT, distanceBytes, 4)) {
        validReading = false;
        return;
    }
    
    // Read signal strength
    uint8_t strengthBytes[2];
    readRegister(REG_SIGNAL_STRENGTH, strengthBytes, 2);
    signalStrength = (strengthBytes[0] << 8) | strengthBytes[1];
    
    // Convert bytes to float (distance in meters)
    float distance;
    memcpy(&distance, distanceBytes, 4);
    
    // Validate reading
    if (validateReading(distance, signalStrength)) {
        applyFilter(distance);
        validReading = true;
    } else {
        validReading = false;
    }
}

void LidarSensor::applyFilter(float newReading) {
    // Update moving average filter
    total -= readings[readIndex];
    readings[readIndex] = newReading;
    total += readings[readIndex];
    readIndex = (readIndex + 1) % FILTER_SIZE;
    
    currentAltitude = total / FILTER_SIZE;
}

bool LidarSensor::validateReading(float distance, uint16_t strength) {
    // Check if distance is within valid range
    if (distance < MIN_RANGE_M || distance > MAX_RANGE_M) {
        return false;
    }
    
    // Check signal strength (threshold may need adjustment based on conditions)
    if (strength < 100) {  // Minimum acceptable signal strength
        return false;
    }
    
    return true;
}

float LidarSensor::getAltitude() const {
    return currentAltitude;
}

bool LidarSensor::isValidReading() const {
    return validReading;
}

uint16_t LidarSensor::getSignalStrength() const {
    return signalStrength;
}

void LidarSensor::setUpdateRate(uint16_t rate) {
    rate = constrain(rate, 48, 5000);  // LW20 supports 48-5000Hz
    uint8_t rateBytes[2] = {
        static_cast<uint8_t>(rate >> 8),
        static_cast<uint8_t>(rate & 0xFF)
    };
    writeRegister(REG_UPDATE_RATE, rateBytes, 2);
}

void LidarSensor::setResolution(uint8_t res) {
    res = constrain(res, 1, 10);  // 1-10cm resolution
    writeRegister(REG_RESOLUTION, &res, 1);
}

void LidarSensor::setStreamMode(bool enable) {
    uint8_t mode = enable ? 1 : 0;
    writeRegister(REG_STREAM_MODE, &mode, 1);
    streamModeEnabled = enable;
}

bool LidarSensor::readRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(LW20_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    
    Wire.requestFrom(LW20_ADDRESS, len);
    if (Wire.available() != len) {
        return false;
    }
    
    for (uint8_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    
    return true;
}

bool LidarSensor::writeRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(LW20_ADDRESS);
    Wire.write(reg);
    Wire.write(data, len);
    return (Wire.endTransmission() == 0);
} 