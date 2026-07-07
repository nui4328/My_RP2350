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
#define TCS34725_ADDRESS 0x29   // ปกติคือ 0x29 (ถ้าเป็น 0x39 ให้เปลี่ยนตรงนี้)
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

void setup() {
  
    opt_begin();
    imu.begin();
    //imu.update();
    imu.update_gyro();
    imu.resetAngles();
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

    trim_servo_0(-10);    // arm_hand ซ้าย       ---> เพิ่มหรือลดตัวเลขในวงเล็บ
    trim_servo_1(0);    // arm_shoulder  ซ้าย 
    trim_servo_2(0);    // arm_shoulder  ขวา  
    trim_servo_3(0);    // arm_hand ขวา
    trim_servo_4(0);    // arm_can ซ้าย  5
    trim_servo_5(0);    // arm_can ขวา

    set_PULSES_updown(100);        // ตั้งค่า ewncoder ระยะขึ้นลงของแขน

    set_PULSES_moveFW(450);     // ตั้งค่า ewncoder ระยะเดินหน้า
    set_PULSES_moveBW(450);      // ตั้งค่า ewncoder ระยะถอยหลัง

    // ตั้งค่าก่อนเรียก turnGyro
    set_move_pid(2.35, 0.015, 0.042);   // PID สำหรับแข่ง
    

    // ตั้งค่าแรงมอเตอร์ แยกซ้าย-ขวา
    set_turnL_motor_bias(1.0, 1.0);   // หมุนซ้าย: มอเตอร์ A แรงขึ้น, มอเตอร์ C อ่อนลง//set_turnL_motor_bias(0.6, 1.2);   // หมุนซ้าย: มอเตอร์ A อ่อนลง, มอเตอร์ C แรงขึ้น
    set_turnR_motor_bias(1.0, 1.0);   // หมุนขวา: มอเตอร์ A อ่อนลง, มอเตอร์ C แรงขึ้น

    Set_distanceFW_to_turnGyro(110);  //---->> ตั้งค่าระยะเดินออกจากเส้นก่อนหมุนตัวหลังจากใช้ คำสั่งsetrobot_bw(2);
    Set_distanceBW_to_turnGyro(115);  //---->> ตั้งค่าระยะเดินออกจากเส้นก่อนหมุนตัวหลังจากใช้ คำสั่งsetrobot_fw(2);
    set_pid_turnL(3.75, 0.0012, 0.175);
    set_pid_turnR(3.75, 0.0012, 0.175);    

    servo_begin();   //------>> เริ่มต้นมือจับ  มือคีบชนกัน   แขนชี้ไปข้างหน้า
    selectAndRunMode();
    robot.loadCalibration();
    robot.playTone(3000, 300);

    

  // turnGyro(90, -90); หมุ่นซ้าย
  // turnGyro(90, 90); หมุ่นขวา
  //-------------------------------------------------------------------------------------------------------------------------------------// เขียนโค้ดที่นี้
  // //-------------------------------------------------------------------------------------------------------------------------------------//
    
    // turnGyro(70, -90);
    // turnGyro(70, 90);
    // sw();
    // move(80,  80, 1.5, 57, 15 , 0); 
    // turnGyro(70, -90);
    // setrobot_bw(3);
   

    // move_bridge(50,  50, 1.5, 60, 15 , 0);    /////////////////////////// สะพาน
    // turnGyro(70, -90);
    // setrobot_fw(2);
    // turnGyro(70, 95);
    // move_bridge(50,  50, 1.5, 82, 15 , 0);    /////////////////////////// สะพาน
    // turnGyro(70, -90);

    // move(-60,  -60, 1.5, 25, 0 , 0);  
    // setrobot_bw(2);
    // turnGyro(70, -90);
    
    // move(-60,  -60, 1.5, 25, 0 , 0); 
    // setrobot_bw(2);
    // turnGyro(70, 90);
    // setrobot_bw(2);

    // move(80,  80, 1.5, 23, 15 , 0); 
    // turnGyro(70, -90);
    // setrobot_bw(2);

    
    // move_bridge(50,  50, 1.5, 115, 15 , 0);    /////////////////////////// สะพาน
    // turnGyro(70, 90);
    // setrobot_fw(2);
    // turnGyro(70, -90);

    // move_bridge(50,  50, 1.5, 50, 0 , 0);    /////////////////////////// สะพาน
    // setrobot_fw(2);
    // turnGyro(70, 90);

    // move(80,  80, 1.5, 50, 0 , 0); 
    // setrobot_fw(2);

    // turnGyro(70, 90);
    // move(80,  80, 1.5, 20, 15 , 0);   
    // turnGyro(70, 90);
    // setrobot_fw(2);
    // turnGyro(70, -90);
    // delay(500);
    // move_chopsticks(30,  30, 1.5, 55, 5, 0);  //----------->> ข้ามตะเกียบ
    // move(50,  50, 1.5, 8, 0 , 0);   

    // setrobot_fw(2);
    // turnGyro(70, -90);
    // setrobot_fw(2);
    // move(-50,  -50, 1.5, 28, 15 , 0); 

    // turnGyro(70, -90);
    // move(80,  80, 1.5, 80, 15 , 0); 
    // setrobot_fw(2);
    // turnGyro(70, 90);
    // move(80,  80, 1.5, 26, 15 , 0); 


























    // arm_updown(0);
    // arm_shoulder('L', 65);
    // arm_shoulder('R', 65);
    // arm_hand('L', 80);
    // arm_hand('R', 80);
    // move_untrasonic(25, 25, 0.5, 5);  //----->>เดินเข้าไปคีบ
    // delay(200);
    // arm_hand('L', 40);
    // arm_hand('R', 40);
    // delay(100);

    // move(-30,  -30, 2.4, 25, 2); 
    // setrobot_bw(3);
 
    // turnGyro(70, 90);
  // setrobot_bw(3);

  // arm_updown(0);
  // arm_shoulder('L', 65);  // หัวไหล่ชี้ไปข้างหน้า
  // arm_shoulder('R', 65);  // หัวไหล่ชี้ไปข้างหน้า
  // arm_hand('L', 90);      // กางมือออก
  // arm_hand('R', 90);      // กางมือออก

  // //move_adc(15, 1.5, 700); 
  // move(20, 20, 2.5, 30, 1);

  // move(-50, -50, 2.5, 30, 1);
  // turnGyro(70, -90);
  


 











  // sw();
  // arm_updown(20);
  // delay(500);
  // turnGyro(70, -90);
  // delay(100);
  // turnGyro(70, 90);
  // move(50, 50, 2.5, 20, 3);
  // sw();

  // move(50, 50, 2.5, 65, 5);
  // turnGyro(70, 90);
  // arm_hand('L', 70);
  // arm_hand('R', 70);
  // setrobot_bw(3);
  // turnGyro(70, -90);
  // move(15, 15, 2.5, 10, 5);
  // delay(100);
  // arm_hand('L', 30);
  // arm_hand('R', 30);
  // delay(200);
  // arm_updown(7);
  // move(-30, -30, 2.5, 10, 7);
  // move(-40, -40, 2.5, 40, 5);
  // turnGyro(70, 90);
  // setrobot_bw(3);




  // delay(1000);
  // turnGyro(70, 90);

  //-------------------------------------------------------------------------------------------------------------------------------------//
  //-------------------------------------------------------------------------------------------------------------------------------------//
}

void loop() {

   imu.update_gyro();
   Serial.print(imu.yawRaw());
   Serial.print("   ");
        Serial.println(imu.yaw());delay(10);

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
  // Serial.print(gyro.gyro('z'), 1);
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