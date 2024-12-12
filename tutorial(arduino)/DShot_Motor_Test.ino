#include <driver/rmt.h>
#include <esp_err.h>

// Motor pins
const uint8_t MOTOR1_PIN = 25;  // Front Right
const uint8_t MOTOR2_PIN = 26;  // Front Left
const uint8_t MOTOR3_PIN = 27;  // Rear Left
const uint8_t MOTOR4_PIN = 33;  // Rear Right

// RMT configuration
const uint8_t NUM_MOTORS = 4;
const uint8_t RMT_CLK_DIV = 2;  // 80MHz / 2 = 40MHz (25ns resolution)

// DShot timing (in RMT ticks at 40MHz)
const uint16_t BIT1_HIGH = 100;  // 2.5µs
const uint16_t BIT1_LOW = 33;    // 0.833µs
const uint16_t BIT0_HIGH = 50;   // 1.25µs
const uint16_t BIT0_LOW = 83;    // 2.083µs

// Global variables
rmt_config_t rmt_configs[NUM_MOTORS];
rmt_channel_t channels[NUM_MOTORS] = {
    RMT_CHANNEL_0,
    RMT_CHANNEL_1,
    RMT_CHANNEL_2,
    RMT_CHANNEL_3
};
const uint8_t motorPins[NUM_MOTORS] = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN};

// Function declarations
bool setupRMTChannel(uint8_t motor_index, uint8_t pin);
void armESCs();
uint16_t prepareDshotPacket(uint16_t value, bool telemetry = false);
void sendDshotPacket(uint8_t motor_index, uint16_t value);
void setMotorSpeed(uint8_t motor_index, float speed);

void setup() {
    Serial.begin(115200);
    Serial.println("DShot Motor Test Starting...");
    
    // Initialize RMT channels for each motor
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!setupRMTChannel(i, motorPins[i])) {
            Serial.printf("Failed to initialize RMT channel %d\n", i);
            while (1) delay(100);
        }
    }
    
    // Arm ESCs
    armESCs();
    Serial.println("Motors armed and ready!");
    Serial.println("Enter motor number (1-4) and speed (0-100) in format: 'motor,speed'");
    Serial.println("Example: '1,50' sets motor 1 to 50% speed");
    Serial.println("Enter '0,0' to stop all motors");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        int comma = input.indexOf(',');
        if (comma != -1) {
            int motor = input.substring(0, comma).toInt();
            int speed = input.substring(comma + 1).toInt();
            
            if (motor == 0) {  // Stop all motors
                for (int i = 0; i < NUM_MOTORS; i++) {
                    setMotorSpeed(i, 0);
                }
                Serial.println("All motors stopped");
            } else if (motor >= 1 && motor <= 4 && speed >= 0 && speed <= 100) {
                setMotorSpeed(motor - 1, speed / 100.0f);
                Serial.printf("Motor %d set to %d%%\n", motor, speed);
            } else {
                Serial.println("Invalid input. Use format: motor(1-4),speed(0-100)");
            }
        }
    }
}

bool setupRMTChannel(uint8_t motor_index, uint8_t pin) {
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
    if (result != ESP_OK) return false;
    
    result = rmt_driver_install(config.channel, 0, 0);
    if (result != ESP_OK) return false;
    
    rmt_configs[motor_index] = config;
    return true;
}

void armESCs() {
    Serial.println("Arming ESCs...");
    // Send zero throttle for 1 second
    for (int i = 0; i < 1000; i++) {
        for (int motor = 0; motor < NUM_MOTORS; motor++) {
            sendDshotPacket(motor, 0);
        }
        delay(1);
    }
    Serial.println("ESCs armed!");
}

uint16_t prepareDshotPacket(uint16_t value, bool telemetry) {
    value = constrain(value, 0, 2047);
    
    uint16_t packet = value << 1;
    if (telemetry) {
        packet |= 1;
    }
    
    // Calculate checksum
    uint16_t csum = 0;
    uint16_t csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    
    return (packet << 4) | csum;
}

void sendDshotPacket(uint8_t motor_index, uint16_t value) {
    uint16_t packet = prepareDshotPacket(value, false);
    rmt_item32_t rmt_items[16];
    
    for (int i = 0; i < 16; i++) {
        bool bit = (packet >> (15 - i)) & 1;
        if (bit) {
            rmt_items[i].level0 = 1;
            rmt_items[i].duration0 = BIT1_HIGH;
            rmt_items[i].level1 = 0;
            rmt_items[i].duration1 = BIT1_LOW;
        } else {
            rmt_items[i].level0 = 1;
            rmt_items[i].duration0 = BIT0_HIGH;
            rmt_items[i].level1 = 0;
            rmt_items[i].duration1 = BIT0_LOW;
        }
    }
    
    rmt_write_items(channels[motor_index], rmt_items, 16, true);
    rmt_wait_tx_done(channels[motor_index], portMAX_DELAY);
}

void setMotorSpeed(uint8_t motor_index, float speed) {
    speed = constrain(speed, 0.0f, 1.0f);
    // Convert 0-1 float to 48-2047 DShot range
    uint16_t dshot_value = 48 + static_cast<uint16_t>(speed * 1999);
    sendDshotPacket(motor_index, dshot_value);
} 