#include "motor_control.h"
#include <algorithm>

MotorControl::MotorControl() {
    channels[0] = RMT_CHANNEL_0;
    channels[1] = RMT_CHANNEL_1;
    channels[2] = RMT_CHANNEL_2;
    channels[3] = RMT_CHANNEL_3;
}

bool MotorControl::init() {
    // Setup RMT channels for each motor
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!setupRMTChannel(i, motorPins[i])) {
            Serial.printf("Failed to initialize RMT channel %d\n", i);
            return false;
        }
    }
    
    // Arm ESCs with DShot protocol
    armESCs();
    
    return true;
}

bool MotorControl::setupRMTChannel(uint8_t motor_index, uint8_t pin) {
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = channels[motor_index];
    config.gpio_num = static_cast<gpio_num_t>(pin);
    config.mem_block_num = 1;
    config.clk_div = RMT_CLK_DIV;
    
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    
    esp_err_t result = rmt_config(&config);
    if (result != ESP_OK) {
        return false;
    }
    
    result = rmt_driver_install(config.channel, 0, 0);
    if (result != ESP_OK) {
        return false;
    }
    
    rmt_configs[motor_index] = config;
    return true;
}

void MotorControl::armESCs() {
    // Send zero throttle for 1 second to arm ESCs
    Serial.println("Arming ESCs...");
    for (int i = 0; i < 1000; i++) {
        for (int motor = 0; motor < NUM_MOTORS; motor++) {
            sendDshotPacket(motor, 0);
        }
        delay(1);
    }
    Serial.println("ESCs armed!");
}

void MotorControl::updateMotors(float m1, float m2, float m3, float m4) {
    static float lastMotors[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float motors[4] = {m1, m2, m3, m4};
    
    // Apply rate limiting to prevent sudden changes
    const float MAX_CHANGE = 0.1f;  // Maximum 10% change per update
    for (int i = 0; i < NUM_MOTORS; i++) {
        float change = motors[i] - lastMotors[i];
        if (abs(change) > MAX_CHANGE) {
            motors[i] = lastMotors[i] + (change > 0 ? MAX_CHANGE : -MAX_CHANGE);
        }
        lastMotors[i] = motors[i];
    }
    
    // Send DShot commands to all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        constrainMotorOutput(motors[i]);
        // Convert 0-1 float to 48-2047 DShot range (leaving space for special commands 0-47)
        uint16_t dshot_value = 48 + static_cast<uint16_t>(motors[i] * 1999);
        sendDshotPacket(i, dshot_value);
    }
}

void MotorControl::emergencyStop() {
    // Send DShot motor_stop command (0) to all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        sendDshotPacket(i, 0);
    }
}

void MotorControl::constrainMotorOutput(float& output) {
    output = std::max(0.0f, std::min(1.0f, output));
}

uint16_t MotorControl::prepareDshotPacket(uint16_t value, bool telemetry) {
    // Ensure the value is within the valid range
    value = std::min(value, static_cast<uint16_t>(2047));
    
    uint16_t packet = value << 1;
    if (telemetry) {
        packet |= 1;
    }
    
    // Calculate checksum (last 4 bits)
    uint16_t csum = 0;
    uint16_t csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    
    return (packet << 4) | csum;
}

void MotorControl::sendDshotPacket(uint8_t motor_index, uint16_t value) {
    uint16_t packet = prepareDshotPacket(value, false);
    
    // Convert to RMT items
    rmt_item32_t rmt_items[16];
    
    for (int i = 0; i < 16; i++) {
        bool bit = (packet >> (15 - i)) & 1;
        if (bit) {
            // DShot600 "1" bit: 2.5µs high, 0.833µs low
            rmt_items[i].level0 = 1;
            rmt_items[i].duration0 = 100; // 2.5µs at 40MHz
            rmt_items[i].level1 = 0;
            rmt_items[i].duration1 = 33;  // 0.833µs at 40MHz
        } else {
            // DShot600 "0" bit: 1.25µs high, 2.083µs low
            rmt_items[i].level0 = 1;
            rmt_items[i].duration0 = 50;  // 1.25µs at 40MHz
            rmt_items[i].duration1 = 83;  // 2.083µs at 40MHz
            rmt_items[i].level1 = 0;
        }
    }
    
    // Wait for previous transmission to complete
    rmt_wait_tx_done(channels[motor_index], portMAX_DELAY);
    
    // Send the packet
    esp_err_t result = rmt_write_items(channels[motor_index], rmt_items, 16, true);
    if (result != ESP_OK) {
        Serial.printf("Failed to send DShot packet to motor %d\n", motor_index);
    }
}

// Special DShot commands
void MotorControl::sendSpecialCommand(uint16_t command) {
    // Commands 0-47 are special commands in DShot protocol
    if (command <= 47) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            sendDshotPacket(i, command);
        }
    }
} 