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

int trim_0, trim_1, trim_2, trim_3, trim_4, trim_5;
void move(int sl, int sr, float kp, float dist, int offset, float arm_target, float deg = degree_turnGyro);
void move_chopsticks(int sl, int sr, float kp, float dist, int offset, float arm_target, float deg = degree_turnGyro);
void move_bridge(int sl, int sr, float kp, float dist, int offset, float arm_target, float deg = degree_turnGyro);
void move_untrasonic_to_stand(int sl, int sr, float kp, float untra, float deg = degree_turnGyro);
void move_untrasonic(int sl, int sr, float kp, float untra, float deg = degree_turnGyro);
void arm_updown(float target_cm);
void arm_updown_nonblocking(float target_cm);
void arm_update_nonblocking();
void trim_servo_0(int trim);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////// ตั้งค่า
int arm_shoulderL_begin = 78;  //---------------->>  หัวไหล่ซ้าย  ชี้ไปข้างหน้า เริ่มต้น
int arm_shoulderR_begin = 70;  //---------------->>  หัวไหล่ขวา  ชี้ไปข้างหน้า เริ่มต้น

void setup() {
  opt_begin();

  //-------------------------------------------------------ปรับทิมเซอร์โว
  trim_servo_0(-15);  // arm_hand ซ้าย   ค่ามากกางออก
  trim_servo_1(0);    // arm_shoulder  ซ้าย
  trim_servo_2(-3);   // arm_shoulder  ขวา   ค่าน้อยหุบเข้า
  trim_servo_3(0);    // arm_hand ขวา   ค่ามากกางออก
  trim_servo_4(6);    // arm_can ซ้าย  ค่าน้อยไปข้างหน้า ค่ามากไปข้างหลัง
  trim_servo_5(5);    // arm_can ขวา   ค่าน้อยไปข้างหน้า

  set_PULSES_updown(110);           // ตั้งค่า ewncoder ระยะขึ้นลงของแขน
  Set_distanceFW_to_turnGyro(95);   //---->> ตั้งค่าระยะเดินออกจากเส้นก่อนหมุนตัวหลังจากใช้ คำสั่งsetrobot_bw(2);
  Set_distanceBW_to_turnGyro(115);  //---->> ตั้งค่าระยะเดินออกจากเส้นก่อนหมุนตัวหลังจากใช้ คำสั่งsetrobot_fw(2);

  set_pid_turnL(1.2f, 0.0001f, 0.042f);  //-->>  ตั้งค่า pid ในการหมุนตัว
  set_pid_turnR(1.2f, 0.0001f, 0.042f);  //  หมุนแรงน้อยไปไม่ถึง เพิ่มค่าตัวแรก
                                           //  หมุนแรงมากไปเลย ลดค่าตัวแรก



  //  arm_ready();       // แขนทั้งหมดเตรียมพร้อม หัวไหลชี้ไปข้างหน้า กางมือออก
  //set_hand_zero();       // ตั้งค่ามือจับ ให้ปลายชิดกับ
  // set_arm_to_can();     // เปิดคอมเมนต์ ตอนประกอบครั้งแรกจัด มือ ให้ตรงกับ ที่เก็บ  ทั้ง 2 ข้าง

  selectAndRunMode();
  robot.loadCalibration();
  robot.playTone(3000, 300);
  imu.update();
  imu.resetAngles();

  // เริ่มต้นเขียนโปรแกรม
  //-------------------------------------------------------------------------------------------------------------------------------------// เขียนโค้ดที่นี้
  //-------------------------------------------------------------------------------------------------------------------------------------//

  
  //  move_bridge(70, 70, 1.5, 30, 10, 0 );
  //  turnGyro(70, 90, true);  //----->>หมุนตามทิศ
  //  //setrobot_bw(1);         //----->>ถอยหลังเซต,


  move_bridge(70, 70, 1.5, 60, 10, 0 );
  turnGyro(70, 90, true);
  setrobot_bw(1);         //----->>ถอยหลังเซต,

  
  move_bridge(50, 50, 1.5, 35, 10, 0 );
  turnGyro(70, 0, true);

  move_bridge(50, 50, 1.5, 30, 10, 0 );
  setrobot_fw(1);         //----->>ถอยหลังเซต,

  move_bridge(-50, -50, 1.5, 65, 10, 0 );
  turnGyro(70, 90, true);


   
  //-------------------------------------------------------------------------------------------------------------------------------------//
  //-------------------------------------------------------------------------------------------------------------------------------------//
}

void loop() {

   imu.update();
   Serial.println(imu.yaw());delay(10);

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




  // for(int i=0; i<10; i++)
  //   {
  //     Serial.print(robot.adcRead(i));
  //     Serial.print(" | ");
  //   }
  //   Serial.println();
  //   delay(10);
  //Serial.println(robot.knopRead());
}