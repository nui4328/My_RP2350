#include <ArrayPico2.h>
#include <EncoderLibrarys.h>
#include <my_BMI160.h>
#include "MoveGyroPID.h"
EncoderLibrarys encoder(20, 19, 22, 21);
ArrayPico2 robot;
my_BMI160 gyro; // สร้างอ็อบเจ็กต์ด้วยที่อยู่เริ่มต้น (0x69)

void setup() 
  {
    robot.begin();  // เริ่มต้นทุกอย่าง
    setup_robot();

    Set_travel_distance_toturnGyro (130); //------>> ตั้งค่า เคลื่อนที่ออกจากเส้นก่อนหมุนตัว
    set_pid_moveL(1.55, 0.005, 0.05);   //------->> ตั้งค่า pid สำหรับหมุนซ้าย    ให้ปรับตัวแปรตัวสุดท้ายตัวเดียว  ถ้าหมุนเลยองศาให้เพิ่มขึ้น     **************************
    set_pid_moveR(1.35, 0.005, 0.05);   //------->> ตั้งค่า pid สำหรับหมุนขวา    ให้ปรับตัวแปรตัวสุดท้ายตัวเดียว  ถ้าหมุนเลยองศาให้เพิ่มขึ้น     **************************
    Serial.print("gyro.gyro('z') :");
  //------------------------------------------------------------------------>>>>
  //------------------------------------------------------------------------>>>>  เขียนโค๊ดต่าง ๆ ที่นี้
   
   
   box1();
   box2();
   checkpoint_1();
   box3();








 
  //------------------------------------------------------------------------>>>>
  //------------------------------------------------------------------------>>>>
  //------------------------------------------------------------------------>>>>
    
  }

void loop() 
  { 

    
    /*
    for(int i=0; i<9; i++)
      {
        Serial.print(robot.adcRead(i)); 
        Serial.print("   "); 
      }
    Serial.println(""); 
  */
        //Serial.println(robot.adcRead(0));
    //Serial.print(encoder.Poss_L());Serial.print("  "); Serial.println(encoder.Poss_R());
    Serial.print("gyro.gyro('z') :");Serial.print(gyro.gyro('z')); Serial.println( "   " ); 
    delay(10);
  }