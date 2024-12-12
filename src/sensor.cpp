#include "sensor.h"
#include <math.h>

SensorSystem::SensorSystem()
    : roll(0.0f)
    , pitch(0.0f)
    , yaw(0.0f)
    , complementaryFilterCoeff(0.96f)
    , isInitialized(false) {
}

bool SensorSystem::init() {
    Wire.begin();
    Wire.setClock(400000);  // Set I2C clock to 400kHz
    
    if (!mpu.begin()) {
        Serial.println("MPU6050 initialization failed!");
        return false;
    }
    
    // Configure MPU6050
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // Perform initial calibration
    calibrateSensors();
    
    isInitialized = true;
    return true;
}

void SensorSystem::calibrateSensors() {
    Serial.println("Calibrating sensors... Keep drone still!");
    
    float gyroXsum = 0, gyroYsum = 0, gyroZsum = 0;
    const int numSamples = 1000;
    
    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        gyroXsum += g.gyro.x;
        gyroYsum += g.gyro.y;
        gyroZsum += g.gyro.z;
        
        delay(1);
    }
    
    gyroXOffset = gyroXsum / numSamples;
    gyroYOffset = gyroYsum / numSamples;
    gyroZOffset = gyroZsum / numSamples;
    
    Serial.println("Calibration complete!");
}

void SensorSystem::update() {
    if (!isInitialized) return;
    
    sensors_event_t a, g, temp;
    if (!mpu.getEvent(&a, &g, &temp)) {
        Serial.println("Failed to read MPU6050 data!");
        return;
    }
    
    // Apply calibration offsets
    g.gyro.x -= gyroXOffset;
    g.gyro.y -= gyroYOffset;
    g.gyro.z -= gyroZOffset;
    
    // Calculate attitude using complementary filter
    calculateAttitude(a.acceleration.x, a.acceleration.y, a.acceleration.z,
                     g.gyro.x, g.gyro.y, g.gyro.z);
}

void SensorSystem::calculateAttitude(float accelX, float accelY, float accelZ,
                                   float gyroX, float gyroY, float gyroZ) {
    // Calculate attitude from accelerometer
    float accelRoll = atan2(accelY, accelZ) * RAD_TO_DEG;
    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;
    
    // Integrate gyro data
    static unsigned long lastUpdate = 0;
    unsigned long now = micros();
    float dt = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;
    
    // Apply complementary filter
    roll = complementaryFilterCoeff * (roll + gyroX * dt) +
           (1.0f - complementaryFilterCoeff) * accelRoll;
    
    pitch = complementaryFilterCoeff * (pitch + gyroY * dt) +
            (1.0f - complementaryFilterCoeff) * accelPitch;
    
    yaw += gyroZ * dt;
    
    // Normalize yaw to 0-360 degrees
    if (yaw < 0) yaw += 360;
    else if (yaw >= 360) yaw -= 360;
}

float SensorSystem::getRoll() const { return roll; }
float SensorSystem::getPitch() const { return pitch; }
float SensorSystem::getYaw() const { return yaw; }

bool SensorSystem::isHealthy() const {
    return isInitialized && mpu.getEvent(nullptr, nullptr, nullptr);
} 