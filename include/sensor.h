#ifndef SENSOR_H
#define SENSOR_H

#include <Wire.h>
#include <MPU6050.h>

class SensorSystem {
public:
    SensorSystem();
    bool init();
    void update();
    
    // Getter methods for processed sensor data
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;
    
    bool isHealthy() const {
        return isInitialized && mpu.getEvent(nullptr, nullptr, nullptr);
    }
    
private:
    MPU6050 mpu;
    float roll, pitch, yaw;
    float complementaryFilterCoeff;
    bool isInitialized;
    float gyroXOffset, gyroYOffset, gyroZOffset;
    
    void calculateAttitude(float accelX, float accelY, float accelZ, 
                          float gyroX, float gyroY, float gyroZ);
};

#endif 