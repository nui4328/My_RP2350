#pragma once

#include <Arduino.h>

namespace MyMINIConfig {

// Raspberry Pi Pico 2 GPIO numbers (Arduino-Pico core).
constexpr uint8_t PIN_START_BUTTON = 2;
constexpr uint8_t PIN_FRONT_CAL = 3;  // Active LOW from front sensor board.
constexpr uint8_t PIN_I2C_SDA = 4;
constexpr uint8_t PIN_I2C_SCL = 5;
constexpr uint8_t PIN_BUZZER = 9;

constexpr uint8_t FRONT_MUX_SELECT[4] = {17, 16, 15, 14}; // S0..S3
constexpr uint8_t FRONT_MUX_SIGNAL = 27;
constexpr uint8_t REAR_MUX_SELECT[4] = {10, 11, 12, 13};  // S0..S3
constexpr uint8_t REAR_MUX_SIGNAL = 26;

constexpr uint8_t LEFT_MOTOR_PWM = 6;
constexpr uint8_t LEFT_MOTOR_IN1 = 8;
constexpr uint8_t LEFT_MOTOR_IN2 = 7;
constexpr uint8_t RIGHT_MOTOR_PWM = 19;
constexpr uint8_t RIGHT_MOTOR_IN1 = 21;
constexpr uint8_t RIGHT_MOTOR_IN2 = 20;

// Optional DRV8874 replacement for the two TB6612FNG modules. These four
// PH/EN and nSLEEP signals are confirmed for the installed two-module DRV8874
// replacement. Each nSLEEP is Pico-controlled; neither is hard-wired high.
constexpr uint8_t DRV8874_LEFT_ENABLE_PWM = 19;
constexpr uint8_t DRV8874_LEFT_PHASE = 20;
constexpr uint8_t DRV8874_RIGHT_ENABLE_PWM = 6;
constexpr uint8_t DRV8874_RIGHT_PHASE = 7;
constexpr uint8_t DRV8874_PIN_UNASSIGNED = 0xFF;
constexpr bool DRV8874_NSLEEP_HARDWIRED_HIGH = false;
constexpr uint8_t DRV8874_LEFT_NSLEEP = 21;
constexpr uint8_t DRV8874_RIGHT_NSLEEP = 8;
constexpr bool DRV8874_LEFT_MOTOR_INVERTED = false;
constexpr bool DRV8874_RIGHT_MOTOR_INVERTED = false;

// Change only after a wheel-off-ground direction test.
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = false;

constexpr uint8_t ADS1115_ADDRESS = 0x48;
constexpr uint8_t PCF8574_ADDRESS = 0x20;
constexpr uint8_t CAT24C128_ADDRESS = 0x50;
constexpr uint8_t MCP3421_ADDRESS = 0x68;

constexpr uint8_t SENSOR_COUNT = 16;
constexpr uint8_t ADC_BITS = 12;
constexpr uint16_t MUX_SETTLE_US = 40;
constexpr uint8_t MUX_DISCARD_READS = 1;
constexpr uint8_t MUX_AVERAGE_SAMPLES = 2;
constexpr uint8_t MUX_SMOOTHING_DIVISOR = 4;

constexpr float SENSOR_PITCH_MM = 6.0f;
constexpr float FRONT_SENSOR_Y_MM = 90.0f;
constexpr float REAR_SENSOR_Y_MM = -70.0f;
constexpr float SENSOR_HEIGHT_MM = 5.0f;
constexpr float TRACK_LINE_WIDTH_MM = 20.0f;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t I2C_CLOCK_HZ = 400000;

constexpr uint16_t MOTOR_COMMAND_MAX = 100;
constexpr uint16_t MOTOR_DIAGNOSTIC_LIMIT = 30;
constexpr uint16_t MOTOR_PWM_MAX = 4095;
constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ = 15000;
constexpr uint32_t MOTOR_TEST_TIMEOUT_MS = 1000;

// ADS1115 uses +/-4.096 V range. Battery divider is 20 kOhm / 6.8 kOhm.
constexpr float ADS1115_FULL_SCALE_VOLTS = 4.096f;
constexpr float BATTERY_DIVIDER_MULTIPLIER = (20000.0f + 6800.0f) / 6800.0f;
constexpr uint16_t BATTERY_LED_MIN_MV = 11000;
constexpr uint16_t BATTERY_LED_MAX_MV = 12400;
constexpr uint32_t BATTERY_LED_UPDATE_MS = 200;
constexpr uint16_t BATTERY_WARNING_MIN_MV = 6000;
constexpr uint16_t BATTERY_WARNING_TWO_LED_HZ = 1800;
constexpr uint16_t BATTERY_WARNING_ONE_LED_HZ = 3200;
constexpr uint16_t BATTERY_WARNING_ZERO_LED_HZ = 3500;
constexpr uint8_t ADS_CHANNEL_BATTERY = 0;
constexpr uint8_t ADS_CHANNEL_CENTER_LEFT = 1;
constexpr uint8_t ADS_CHANNEL_CENTER_RIGHT = 2;
constexpr uint8_t ADS_CHANNEL_AUX = 3;

// PCF8574 output HIGH turns an LED on.
constexpr bool PCF8574_LED_ACTIVE_LOW = false;

// Reserved only for the explicit destructive-safe EEPROM diagnostic.
constexpr uint16_t EEPROM_TEST_ADDRESS = 0x3FFF;

} // namespace MyMINIConfig
