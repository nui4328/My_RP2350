#ifndef MYRP_PICO2_H
#define MYRP_PICO2_H

#include <Wire.h>
#include <BatteryMonitor.h>
#include <my_BMI160.h>
#include <my_MCP3008s.h>
#include <EncoderLibrarys.h>
#include <Servo.h>
#include <Arduino.h>

#define EEPROM_ADDRESS 0x50
#define PWMA 19
#define AIN1 20
#define AIN2 21
#define PWMB 6
#define BIN1 8
#define BIN2 7

class MyRP_Pico2 {
private:
    BatteryMonitor bat;
    my_MCP3008s adc;
    my_BMI160 my;

    int _lastPosition = 2500;
    float P, I, D, previous_error, errors, PID_output;
    float present_position;
    int setpoint;
    bool pid_error = true;

    bool _fw = true;
    bool setpoint_fw = false;
    bool crossedLine = false;
    bool DC_Motors = true;

    int slmotor = 20, srmotor = 20;
    int clml = -90, clmr = 90, crml = 90, crmr = -90;
    int flml = -15, flmr = 100, frml = 100, frmr = -15;
    int llmotor = -80, lrmotor = 80, ldelaymotor = 8;
    int rlmotor = 80, rrmotor = -80, rdelaymotor = 8;
    int break_ff = 50, break_fc = 50, break_bf = 50, break_bc = 50;
    int delay_f = 20;

    float kd_f = 0.55f, kd_b = 0.025f;
    float kp_slow = 0.1f, ki_slow = 0.0001f;

    float redius_wheel = 3.0f;
    float speed_scale_fw = 1.0f;
    float speed_scale_bw = 1.05f;

    float scale = 1.0f;
    bool batteryUsed = false;

    Servo servo_0, servo_1, servo_10, servo_28;
    int servo_tim0 = 0, servo_tim1 = 0, servo_tim10 = 0, servo_tim28 = 0;
    int num_steps = 20;
    float s10_before_deg = 120, s0_before_deg = 120, s1_before_deg = 50, s28_before_deg = 0;

    int sensor_pin_A[6] = {1,2,3,4,5,6};
    int sensor_pin_B[6] = {1,2,3,4,5,6};

    const float CRUISE_PWM_VAL    = 100.0f;
    const float MIN_PWM_VAL       = 15.0f;
    const float ACCEL_RATE_PWM_S  = 100.0f;
    const float DECEL_RATE_PWM_S  = 100.0f;
    
    const int   ramp_delay        = 6;

    const float VMAX = 12.6f;
    const float VMIN = 7.4f;
    const float VNOM = 11.55f;

    int numSensor = 6;

    // Private helpers    

    void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength);
    void readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength);

    void read_eepA();
    void read_eepB();
    void read_eepC();

    void turn_speed_fl();
    void turn_speed_fr();
    void bturn_speed_fl();
    void bturn_speed_fr();
    

public:
    MyRP_Pico2();
    void begin();

    void calibrateA();
    void calibrateB();
    void calibrateC();
    void loadCalibration();
    void printCalibration();

    void Motor(int left, int right);
    void updateBattery();
    float getBatteryVoltage();
    void bat_control();
    void servo(int num, int angle);
    void armLeftRight(float left, float right, int speed);
    void armUpDown(float angle, int speed);
    void servoTrim(int s0, int s1, int s10, int s28);

    void fline(int spl, int spr, float kp, String target, char nfc = 'f', 
               char splr = ' ', int power = 100, String sensor = "a", int endt = 0);
    
    void bline(int spl, int spr, float kp, String target, char nfc = 'f', 
               char splr = ' ', int power = 100, String sensor = "b", int endt = 0);

    void rotate(int ml, int mr, int degree, int offset = 50);
    
    bool gyroOK();
    void sensor_position(int position);
    int position_A();
    int position_A_none();
    int position_B();
    int position_B_none();
    int _setpoint = 50;

    float error_A();
    float error_AA();
    float error_AN();
    float error_B();
    float error_BB();
    float error_BN();
    // Wrapper สำหรับ setup() ที่คุณใช้
    void set_Freq(String type);
    void distance_scale_fw(float scale);
    void distance_scale_bw(float scale);
    void set_slow_motor(int sl, int sr);
    void set_turn_center_l(int l, int r);
    void set_turn_center_r(int l, int r);
    void set_turn_front_l(int l, int r);
    void set_turn_front_r(int l, int r);
    void set_brake_fc(int ff, int fc);
    void set_brake_bc(int bf, int bc);
    void set_delay_f(int delayTime);

    void setMotorFreq(String type);
    void setSpeed(int sl, int sr);
    void setTurnCenter(int l, int r);
    void setTurnFront(int l, int r);
    void setBrake(int ff, int fc, int bf, int bc);
    void setKD(float kdf, float kdb);
    void setWheelRadius(float r);
    void setSlowPID(float kp, float ki);

    void beep(int freq = 3000, int dur = 100);
    void blink(int times = 3);
    void waitButton();
    void sw();

    void setrobot_fline(int num);
    void setrobot_bline(int num);

    uint16_t read_sensorA(int sensor);
    uint16_t read_sensorB(int sensor);
    int md_sensorA(int sensor);
    int md_sensorB(int sensor);
    int md_sensorC(int sensor);

    void place_left_in(int mr, int degree, int offset);
    void place_left_out(int mr, int degree, int offset);
    void place_right_in(int mr, int degree, int offset);
    void place_right_out(int mr, int degree, int offset);

    int sensorMaxA[8], sensorMinA[8];
    int sensorMaxB[8], sensorMinB[8];
    int sensorMaxC[2], sensorMinC[2];

    void fw_gyro(int spl, int spr, float kp,  float distance, int offset);
    void fw_gyros(int spl, int spr, float kp,  float distance, int offset);
    void bw_gyro(int spl, int spr, float kp,  float distance, int offset);
    void bw_gyros(int spl, int spr, float kp,  float distance, int offset);
    
    float SPEED_FACTOR_CM_Sfw = 2.4;
    void sp_factor_cm_s_forFW(float SP_FACTOR_CM_SS);
    float SPEED_FACTOR_CM_Sbw = 2.4;
    void sp_factor_cm_s_forBW(float SP_FACTOR_CM_SS);
};

#endif