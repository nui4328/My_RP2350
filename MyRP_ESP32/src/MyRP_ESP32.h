#ifndef MYRP_ESP32_H
#define MYRP_ESP32_H

#include <Arduino.h>
#include <Wire.h>
#include "BatteryMonitor.h"
#include "my_BMI160.h"
#include "my_MCP3008s.h"
#include "EncoderLibrarys.h"

class MyRP_ESP32 {
public:
    MyRP_ESP32();
    // ฟังก์ชันหลัก
    void setup_robot();
    void sw();                    // Calibration + Menu
    void updateBattery();
    float getBatteryVoltage();
    // มอเตอร์และเดินตามเส้น
    void Motor(int leftSpeed, int rightSpeed);
    void fw(int sl, int sr, float kp);
    void fline(int spl, int spr, float kp, String target, char nfc, char splr, int power, String sensor, int endt);
    void bline(int spl, int spr, float kp, String target, char nfc, char splr, int power, String sensor, int endt);
    // หมุนด้วย Gyro / ล้อ
    void rotate(int ml, int mr, int degree, int offset);
    void place_left_in(int mr, int degree, int offset);
    void place_left_out(int mr, int degree, int offset);
    void place_right_in(int mr, int degree, int offset);
    void place_right_out(int mr, int degree, int offset);
    void fw_gyro(int spl, int spr, float kp, float distance, int offset);
    void bw_gyro(int spl, int spr, float kp, float distance, int offset);
    void setrobot_fline(int num);
    void setrobot_bline(int num);
    // เซอร์โว
    void servo(int ch, int target_angle);
    void s0_trim(int offset);
    void s1_trim(int offset);
    void s10_trim(int offset);
    void s28_trim(int offset);
    // ตั้งค่าต่าง ๆ
    void set_Freq(String fr_motor);
    void sensor_position(float positionss);
    void set_slow_motor(float sl, float sr);
    void set_turn_center_l(int ml, int mr);
    void set_turn_center_r(int ml, int mr);
    void set_turn_front_l(int ml, int mr);
    void set_turn_front_r(int ml, int mr);
    void set_brake_fc(int ff, int fc);
    void set_brake_bc(int bf, int bc);
    void set_delay_f(int ff);
    void set_speed_turn_fl(int inM, int outM, int delayM);
    void set_speed_turn_fr(int inM, int outM, int delayM);
    void kd_fw(float kd);
    void kd_bw(float kd);
    void kp_sl(float kp_sl, float ki_sl);
    void distance_scale_fw(float scale);
    void distance_scale_bw(float scale);
    void resetAngles();
    void myTone(int pin, int freq, int duration);
    void blinkLED(int count);
    void waitButton();
    // ฟังก์ชันช่วยเหลือ (ประกาศเพียงครั้งเดียว)
    void get_maxMinA();
    void get_maxMinB();
    void get_maxMinC();
    void get_EEP_Program();
    void read_eepA();
    void read_eepB();
    void read_eepC();
    void read_sensorA_program();
    void read_sensorB_program();
    void read_sensorC_program();

    int position_A();
    int position_A_none();
    float error_A();
    float error_AA();
    float error_AN();

    int position_B();
    int position_B_none();
    float error_B();
    float error_BB();
    float error_BN();

    uint16_t read_sensorA(int sensor);
    uint16_t read_sensorB(int sensor);
    int md_sensorA(int sensor);
    int md_sensorB(int sensor);
    int md_sensorC(int sensor);

    // ค่าการตั้งค่าเครื่องยนต์และเบรก
    float slmotor = 20.0f;
    float srmotor = 20.0f;
    int clml = -90, clmr = 90;
    int crml = 90, crmr = -90;
    int flml = -15, flmr = 100;
    int frml = 100, frmr = -15;
    int llmotor = 100, lrmotor = 50, ldelaymotor = 50;
    int rlmotor = 50, rrmotor = 100, rdelaymotor = 50;
    int break_ff = 5, break_fc = 30, break_bf = 10, break_bc = 20;
    int delay_f = 15;
    float kd_f = 0.55f, kd_b = 0.025f;
    float kp_slow = 0.1f, ki_slow = 0.0001f;

    // Servo trim
    int trim0 = 0, trim1 = 0, trim10 = 0, trim28 = 0;
    float last_angle0 = 90.0f, last_angle1 = 90.0f;
    float last_angle10 = 90.0f, last_angle28 = 90.0f;
      // ตัวแปรเซ็นเซอร์
    int sensorValuesA[8][1000];
    int sensorMaxA[8], sensorMinA[8];
    int sensorValuesB[8][1000];
    int sensorMaxB[8], sensorMinB[8];
    int sensorValuesC[2][1000];
    int sensorMaxC[2], sensorMinC[2];

    int sensor_pin_A[6] = {1,2,3,4,5,6};
    int sensor_pin_B[6] = {1,2,3,4,5,6};

    // ตัวแปรควบคุมหลัก
    float P = 0, I = 0, D = 0, previous_I = 0, previous_error = 0, errors = 0, PID_output = 0;
    float present_position = 0, _positions = 50.0f;
    int _lastPosition = 2500;
    bool DC_Motors = true;
    bool pid_error = true;
    bool _fw = true;
    bool setpoint_fw = false;
    bool crossedLine = false;

    float speed_scale_fw = 1.0f;
    float speed_scale_bw = 1.05f;
    // ===== เพิ่มค่าคงที่ตรงนี้ =====
    const float CRUISE_PWM_VAL   = 95.0f;  // ความเร็วสูงสุดที่ใช้ในการวิ่ง
    const float MIN_PWM_VAL      = 15.0f;
    const float ACCEL_RATE_PWM_S = 100.0f;
    const float DECEL_RATE_PWM_S = 100.0f;

private:
    // ไลบรารีภายนอก
    my_BMI160 my;
    my_MCP3008s adc;
    BatteryMonitor bat;
    EncoderLibrarys encoder;

    // PINOUT
    const uint8_t PWMA = 2, AIN1 = 4, AIN2 = 5;
    const uint8_t PWMB = 17, BIN1 = 33, BIN2 = 18;
    const uint8_t BUZZER_PIN = 34, LED_PIN = 48;
    const uint8_t BUTTON1 = 14, BUTTON2 = 13;
    const uint8_t I2C_SDA = 15, I2C_SCL = 16;

    // MCP3008
    const uint8_t MCP_CLK = 40, MCP_MOSI = 42, MCP_MISO = 41;
    const uint8_t MCP_CS_A = 39, MCP_CS_B = 21;

    // Servo pins
    const uint8_t SERVO0_PIN = 11, SERVO1_PIN = 12;
    const uint8_t SERVO10_PIN = 35, SERVO28_PIN = 9;

    // ค่าคงที่
    static const int numSensors = 8;
    static const int numSensorline = 6;
    static const int numSamples = 1000;
    static const int PWM_FREQ = 20000;
    static const int PWM_RES = 10;
    static const uint8_t SERVO_RESOLUTION = 10;
    static const int SERVO_MIN_PW = 500;
    static const int SERVO_MAX_PW = 2500;
    const int ramp_delay = 6;

    // Battery constants
    const float VMAX = 12.6f;
    const float VMIN = 7.4f;
    const float VNOM = 11.75f;

    float scale = 1.0;
    bool batteryUsed = false;

    // EEPROM & MCP3421
    const int EEPROM_ADDRESS = 0x50;
    const int MCP3421_ADDR = 0x68;

    // Servo trim variables
    int servo_tim0  = 0;
    int servo_tim1  = 0;
    int servo_tim10 = 0;
    int servo_tim28 = 0;

    // Servo previous positions
    float s0_before_deg  = 90.0f;
    float s1_before_deg  = 90.0f;
    float s10_before_deg = 90.0f;
    float s28_before_deg = 90.0f;

    int num_steps = 20;   // จำนวนขั้นตอนการเคลื่อนไหวเซอร์โว

    

    // ฟังก์ชันช่วยเหลือภายใน (private)
    

    void setupPWM();
    void setupServoPWM();
    uint32_t angleToDuty(int angle);

    

    int ADC_i2c();

    void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength);
    void readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength);

    // ฟังก์ชัน turn speed
    void turn_speed_fl();
    void turn_speed_fr();
    void bturn_speed_fl();
    void bturn_speed_fr();
};

extern MyRP_ESP32 robot;
// ====================== Global Object ======================

#endif