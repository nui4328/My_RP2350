#include <PiperPico2.h>
#include "EncoderLibrarys.h"
#include "EncoderLibraryss.h"
#include <Wire.h>
#include <my_BMI160.h>
my_BMI160 gyro; 
#include "MoveGyroPID.h"
//#include <UIPiperPico2.h>

//#include <my_BNO085.h>
//my_BNO085 gyro(0x4B, Wire); 



bool Connected_gyro;
PiperPico2 robot;
EncoderLibrarys   encoder1(19, 20, 21, 22);
EncoderLibraryss  encoder2(16, 17, 10, 18);

#include "my_TCS.h"
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
int trim_0, trim_1,trim_2,trim_3,trim_4,trim_5;

void setup() {
  
    opt_begin();

    trim_servo_0(0);    // arm_hand ซ้าย       ---> เพิ่มหรือลดตัวเลขในวงเล็บ
    trim_servo_1(-7);    // arm_shoulder  ซ้าย
    trim_servo_2(10);    // arm_shoulder  ขวา
    trim_servo_3(10);    // arm_hand ขวา
    trim_servo_4(5);    // arm_can ซ้าย
    trim_servo_5(0);    // arm_can ขวา

    set_PULSES_updown(120);        // ตั้งค่า ewncoder ระยะขึ้นลงของแขน

    set_PULSES_moveFW(220);     // ตั้งค่า ewncoder ระยะเดินหน้า
    set_PULSES_moveBW(220);      // ตั้งค่า ewncoder ระยะถอยหลัง

    // ตั้งค่าก่อนเรียก turnGyro
    set_move_pid(2.35, 0.015, 0.042);   // PID สำหรับแข่ง
    

    // ตั้งค่าแรงมอเตอร์ แยกซ้าย-ขวา
    set_turnL_motor_bias(1.0, 1.0);   // หมุนซ้าย: มอเตอร์ A แรงขึ้น, มอเตอร์ C อ่อนลง//set_turnL_motor_bias(0.6, 1.2);   // หมุนซ้าย: มอเตอร์ A อ่อนลง, มอเตอร์ C แรงขึ้น
    set_turnR_motor_bias(1.0, 1.0);   // หมุนขวา: มอเตอร์ A อ่อนลง, มอเตอร์ C แรงขึ้น

    Set_travel_distance_to_turnGyro(90);  //---->> ตั้งค่าระยะเดินออกจากเส้นก่อนหมุนตัวหลังจากใช้ คำสั่งsetrobot_bw(2);
    set_pid_turnL(4.9, 0.0005, 0.165);
    set_pid_turnR(4.9, 0.0005, 0.165);

      

    servo_begin();   //------>> เริ่มต้นมือจับ  มือคีบชนกัน   แขนชี้ไปข้างหน้า
    //arm_can('L',160);
    //arm_can('R',110);

    selectAndRunMode();
    robot.loadCalibration();
    robot.playTone(3000, 300);
    

    arm_can('L', 75);
    arm_shoulder('L', 150);  //----ปรับค่า  
    arm_can('R', 75);
    arm_shoulder('R', 150);  //----ปรับค่า 
    delay(300);
    sw();
    arm_updown(0);
    sw();
    armL_to_can();
    sw();

    s1();

   
 
}

void loop() {

  //Serial.println(robot.adcRead(3));
   Serial.print(encoder1.Poss_L());Serial.print("  ");Serial.print(encoder1.Poss_R());
   Serial.print("  ");
   Serial.print(encoder2.Poss_L());Serial.print("  ");Serial.println(encoder2.Poss_R());
  /*
 if(digitalRead(2) == 0)
    {
      gyro.resetAngles();
    }

 
  Serial.print(gyro.gyro('z'), 1);
  Serial.println(" °");
  delay(10);
  */
  
/*
  for(int i=0; i<10; i++)
    {
      Serial.print(robot.adcMax(i));
      Serial.print(" | ");
    }*/
  //Serial.println(robot.knopRead());
 
  delay(10);
  
}