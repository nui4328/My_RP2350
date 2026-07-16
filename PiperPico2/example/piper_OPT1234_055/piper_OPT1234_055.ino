#include <PiperPico2.h>
#include "EncoderLibrarys.h"
#include "EncoderLibraryss.h"
#include <Wire.h>
#include <my_BMI160.h>
my_BMI160 gyro;
#include "MoveGyroPID.h"
//#include <UIPiperPico2.h>


bool Connected_gyro;
PiperPico2 robot;
EncoderLibrarys encoder1(19, 20, 21, 22);
EncoderLibraryss encoder2(16, 17, 10, 18);

#include "my_TCS.h"
#define TCS34725_ADDRESS 0x29  // ปกติคือ 0x29 (ถ้าเป็น 0x39 ให้เปลี่ยนตรงนี้)
my_TCS tcs0(0x0000);
my_TCS tcs1(0x0040);

// ==================== PROTOTYPES (วางก่อน setup()) ====================
void set_move_pid(float kp, float ki, float kd);
void move(int sl, int sr, float kp, float dist, float arm_target);

extern float move_kp;
extern float move_ki;
extern float move_kd;

// Prototypes
void arm_updown_nonblocking(float target_cm);
void arm_update_nonblocking();

// Global variables อื่น ๆ ที่มีอยู่แล้ว...
int trim_0, trim_1, trim_2, trim_3, trim_4, trim_5;
int break_move = 1;
// ด้านบนสุดของ MoveGyroPID.h ก่อนทุก function
bool hasCan = false;

const int trigPin = 0;
const int echoPin = 1;
long duration;
int distanceCm;
int distanceInch;
//------------------------>> ตัวแปรค่าสีในมือ
String color_armL;
String color_armR;

int arm_shoulderL_begin = 78;  //---------------->>  หัวไหล่ซ้าย  ชี้ไปข้างหน้า เริ่มต้น
int arm_shoulderR_begin = 62;  //---------------->>  หัวไหล่ขวา  ชี้ไปข้างหน้า เริ่มต้น

void move(int sl, int sr, float kp, float dist, int offset, float arm_target, float deg = degree_turnGyro);
void move_chopsticks(int sl, int sr, float kp, float dist, int offset, float arm_target, float deg = degree_turnGyro);
void arm_updown(float target_cm);
void arm_updown_nonblocking(float target_cm);
void arm_update_nonblocking();

void setup() {


  opt_begin();
  imu.begin();
  imu.update();
  imu.resetAngles();
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input

  trim_servo_0(2);   // arm_hand ซ้าย       ---> เพิ่มหรือลดตัวเลขในวงเล็บ  ค่าน้อยกางออก
  trim_servo_1(-2);   // arm_shoulder  ซ้าย
  trim_servo_2(-3);   // arm_shoulder  ขวา   ค่าน้อยหุบเข้า
  trim_servo_3(-3);   // arm_hand ขวา   ค่าน้อยกางออก
  trim_servo_4(-12);  // arm_can ซ้าย  ค่าน้อยไปข้างหน้า
  trim_servo_5(-10);  // arm_can ขวา   ค่ามากไปข้างหน้า

  set_PULSES_updown(105);  // ตั้งค่า ewncoder ระยะขึ้นลงของแขน

  // set_PULSES_moveFW(100);     // ตั้งค่า ewncoder ระยะเดินหน้า
  // set_PULSES_moveBW(400);      // ตั้งค่า ewncoder ระยะถอยหลัง

  // ตั้งค่าก่อนเรียก turnGyro
  set_move_pid(2.35, 0.015, 0.042);  // PID สำหรับแข่ง


  // ตั้งค่าแรงมอเตอร์ แยกซ้าย-ขวา
  set_turnL_motor_bias(1.0, 1.0);  // หมุนซ้าย: มอเตอร์ A แรงขึ้น, มอเตอร์ C อ่อนลง//set_turnL_motor_bias(0.6, 1.2);   // หมุนซ้าย: มอเตอร์ A อ่อนลง, มอเตอร์ C แรงขึ้น
  set_turnR_motor_bias(1.0, 1.0);  // หมุนขวา: มอเตอร์ A อ่อนลง, มอเตอร์ C แรงขึ้น

  Set_distanceFW_to_turnGyro(55);   //---->> ตั้งค่าระยะเดินออกจากเส้นก่อนหมุนตัวหลังจากใช้ คำสั่งsetrobot_bw(2);
  Set_distanceBW_to_turnGyro(115);  //---->> ตั้งค่าระยะเดินออกจากเส้นก่อนหมุนตัวหลังจากใช้ คำสั่งsetrobot_fw(2);
  set_pid_turnL(0.9f, 0.000f, 0.224f);
  set_pid_turnR(0.9f, 0.000f, 0.224f);

  //servo_begin();  //------>> เริ่มต้นมือจับ  มือคีบชนกัน   แขนชี้ไปข้างหน้า
  // arm_can('R', 40);
  arm_ready();

  // arm_shoulder('L', 170);  //----ปรับค่า
  // arm_shoulder('R', 170);  //----ปรับค่า
  // arm_hand('L', 30);  // ฝ่ามือซ้ายหุบเข้า
  // arm_hand('R', 30);
  selectAndRunMode();
  robot.loadCalibration();
  robot.playTone(3000, 300);
  imu.update();
  imu.resetAngles();


  //turnGyro(70, -90);   //หมุ่นซ้าย
  //turnGyro(70, 90);    //หมุ่นขวา
  //-------------------------------------------------------------------------------------------------------------------------------------// เขียนโค้ดที่นี้
  //-------------------------------------------------------------------------------------------------------------------------------------//
    mission_2();
  
  //  turnGyro(70, 90);    //หมุ่นขวา
  //  //delay(500);
  //  turnGyro(70, -90);    //หมุ่นขวา

  //-------------------------------------------------------------------------------------------------------------------------------------//
  //-------------------------------------------------------------------------------------------------------------------------------------//
}

void loop() {
  //  imu.update();
  //  Serial.println(imu.yaw());delay(10);
  Serial.println(untrasonic());delay(10);
  // robot.motor('A', 60);   robot.motor('C', 60);
  // robot.motor('B', 60);   robot.motor('D', 60);

  // delay(1000);
  // robot.motor('A', -60);   robot.motor('C', -60);
  // robot.motor('B', -60);   robot.motor('D', -60);


  // Serial.println(robot.adcRead(6));delay(10);
  // Serial.print("  ");
  // Serial.println(robot.adcRead(4));
  // Serial.print("  ");
  // Serial.print(encoder1.Poss_L());
  // Serial.print("  ");
  // Serial.print(encoder1.Poss_R());
  // Serial.print("  ");
  // Serial.print(encoder2.Poss_L());
  // Serial.print("  ");
  // Serial.println(encoder2.Poss_R()); delay(10);
  /*
 if(digitalRead(2) == 0)
    {
      gyro.resetAngles();
    }

 */
  // Serial.print(gyro.gyro('z'));
  // Serial.println(" °");
  // delay(10);



  // for(int i=0; i<10; i++)
  //   {
  //     Serial.print(robot.adcRead(i));
  //     Serial.print(" | ");
  //   }
  //   Serial.println();
  //   delay(10);
  //Serial.println(robot.knopRead());
}