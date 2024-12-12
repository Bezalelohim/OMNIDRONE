#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <driver/rmt.h>
#include <esp_err.h>

class MotorControl {
public:
    MotorControl();
    bool init();
    void updateMotors(float m1, float m2, float m3, float m4);
    void emergencyStop();
    void sendSpecialCommand(uint16_t command);

private:
    static const uint8_t MOTOR1_PIN = 25;
    static const uint8_t MOTOR2_PIN = 26;
    static const uint8_t MOTOR3_PIN = 27;
    static const uint8_t MOTOR4_PIN = 33;
    
    static const uint8_t NUM_MOTORS = 4;
    static const uint16_t DSHOT600_BIT_TIME = 1667; // nanoseconds (600KHz)
    static const uint16_t DSHOT_FRAME_LENGTH = 16;
    static const uint8_t RMT_CLK_DIV = 2; // 80MHz / 2 = 40MHz (25ns resolution)
    
    const uint8_t motorPins[NUM_MOTORS] = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN};
    rmt_config_t rmt_configs[NUM_MOTORS];
    rmt_channel_t channels[NUM_MOTORS];
    
    void constrainMotorOutput(float& output);
    uint16_t prepareDshotPacket(uint16_t value, bool telemetry = false);
    void sendDshotPacket(uint8_t motor_index, uint16_t value);
    bool setupRMTChannel(uint8_t motor_index, uint8_t pin);
    void armESCs();
    
    // DShot600 timing (in RMT ticks at 40MHz)
    static const uint16_t BIT1_HIGH = 100;  // 2.5µs
    static const uint16_t BIT1_LOW = 33;    // 0.833µs
    static const uint16_t BIT0_HIGH = 50;   // 1.25µs
    static const uint16_t BIT0_LOW = 83;    // 2.083µs
};

// DShot special commands
enum DShot_Command {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEEP1 = 1,
    DSHOT_CMD_BEEP2 = 2,
    DSHOT_CMD_BEEP3 = 3,
    DSHOT_CMD_BEEP4 = 4,
    DSHOT_CMD_BEEP5 = 5,
    DSHOT_CMD_ESC_INFO = 6,
    DSHOT_CMD_SPIN_DIRECTION_1 = 7,
    DSHOT_CMD_SPIN_DIRECTION_2 = 8,
    DSHOT_CMD_3D_MODE_OFF = 9,
    DSHOT_CMD_3D_MODE_ON = 10,
    DSHOT_CMD_SETTINGS_REQUEST = 11,
    DSHOT_CMD_SAVE_SETTINGS = 12,
    DSHOT_CMD_ROTATE_NORMAL = 20,
    DSHOT_CMD_ROTATE_REVERSE = 21,
    DSHOT_CMD_LED0_ON = 22,
    DSHOT_CMD_LED1_ON = 23,
    DSHOT_CMD_LED2_ON = 24,
    DSHOT_CMD_LED3_ON = 25,
    DSHOT_CMD_LED0_OFF = 26,
    DSHOT_CMD_LED1_OFF = 27,
    DSHOT_CMD_LED2_OFF = 28,
    DSHOT_CMD_LED3_OFF = 29
};

#endif 