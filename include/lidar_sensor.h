#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include <Wire.h>

class LidarSensor {
public:
    LidarSensor();
    bool init();
    void update();
    
    // Getter methods
    float getAltitude() const;
    bool isValidReading() const;
    uint16_t getSignalStrength() const;
    
    // Configuration methods
    void setUpdateRate(uint16_t rate);  // Rate in Hz (48-5000)
    void setResolution(uint8_t res);    // Resolution in cm (1-10)
    void setStreamMode(bool enable);     // Enable/disable continuous streaming
    
private:
    static const uint8_t LW20_ADDRESS = 0x66;  // LW20 default I2C address
    static const uint16_t MAX_RANGE_M = 100;   // 100 meters maximum range
    static const float MIN_RANGE_M = 0.2f;     // 0.2 meters minimum range
    
    // LW20 registers and commands
    static const uint8_t REG_PRODUCT_NAME = 0x00;
    static const uint8_t REG_FIRMWARE_VERSION = 0x01;
    static const uint8_t REG_DISTANCE_OUTPUT = 0x02;
    static const uint8_t REG_UPDATE_RATE = 0x03;
    static const uint8_t REG_SIGNAL_STRENGTH = 0x04;
    static const uint8_t REG_RESOLUTION = 0x05;
    static const uint8_t REG_STREAM_MODE = 0x06;
    static const uint8_t REG_NOISE_LEVEL = 0x07;
    
    float currentAltitude;     // in meters
    uint16_t signalStrength;   // Signal strength indicator
    uint8_t noiseLevel;        // Ambient noise level
    bool validReading;
    bool streamModeEnabled;
    
    // Moving average filter parameters
    static const int FILTER_SIZE = 5;
    float readings[FILTER_SIZE];
    int readIndex;
    float total;
    
    bool readRegister(uint8_t reg, uint8_t* data, uint8_t len);
    bool writeRegister(uint8_t reg, uint8_t* data, uint8_t len);
    void applyFilter(float newReading);
    bool validateReading(float distance, uint16_t strength);
};

#endif 