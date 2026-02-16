#include <ArrayPico2.h>
#include <EncoderLibrarys.h>
#include <my_BMI160.h>
#include "MoveGyroPID.h"
EncoderLibrarys encoder(20, 19, 22, 21);
ArrayPico2 robot;
my_BMI160 gyro; // สร้างอ็อบเจ็กต์ด้วยที่อยู่เริ่มต้น (0x69)
#include <my_TCS.h>
#define TCS34725_ADDRESS 0x29   // ปกติคือ 0x29 (ถ้าเป็น 0x39 ให้เปลี่ยนตรงนี้)
// สร้าง instance พร้อม offset แยก
my_TCS tcs0(0x0000);   // เซ็นเซอร์ตัวแรก บน I2C0 (GP4 SDA, GP5 SCL)


bool sett_f, set_bb;
int bw_distance;
int red_box, green_box, blue_box, yello_box, ch_poit;
int servo18 = 90;
int servo17 = 90;
int servo16 = 20;

void setup() 
  {
    robot.ServoWrite(18, servo18);
    robot.ServoWrite(17, servo17);
    robot.ServoWrite(16, servo16);

    robot.begin();  // เริ่มต้นทุกอย่าง
    setup_robot();
    
    Serial.print("gyro.gyro('z') :");

    
    set_pid_moveL(2.4, 0.005, 0.10);   //------->> ตั้งค่า pid สำหรับหมุนซ้าย    ให้ปรับตัวแปรตัวสุดท้ายตัวเดียว  ถ้าหมุนเลยองศาให้เพิ่มขึ้น     **************************
    set_pid_moveR(2.95, 0.005, 0.055);   //------->> ตั้งค่า pid สำหรับหมุนขวา    ให้ปรับตัวแปรตัวสุดท้ายตัวเดียว  ถ้าหมุนเลยองศาให้เพิ่มขึ้น     **************************
    Set_travel_distance_toturnGyro (4); //------>> ตั้งค่า เคลื่อนที่ออกจากเส้นก่อนหมุนตัว
    distance_bw(25);    //----------->> ตั้งค่าระยะถอยหลังเมื่อวางกล่องสี แดง เขียว และน้ำเงิน

  //------------------------------------------------------------------------>>>>
  //------------------------------------------------------------------------>>>>  เขียนโค๊ดต่าง ๆ ที่นี้
   
      // Calibrate เซ็นเซอร์สี
      /*
      calibrateSensor(tcs0, "TCS0 (I2C0)");
      Serial.println("\nการ calibrate เซ็นเซอร์ตรวจจับสีเสร็จสิ้น!");
      Serial.println("ครั้งต่อไปเปิดเครื่องใหม่จะโหลดค่า calibration อัตโนมัติ");
      Serial.println("พร้อมใช้งานแล้ว!");
      robot.run();  
     */
   //-------------------------------------------------------------------------->>>>
    Move_forward_begin(40, 40, 1.2, 30);   //----------->> ออกจากจุดเริ่มต้น
    
    for(int i=0; i<3000; i++)
      {
        Move_forward (40, 40, 1.2, 30);        //--------->>Move_start(30, 30, 1.2, 30);  กำหนดระยะที่เดินแต่ละบล๊อก คือ เลข 30
      }
    



 
  //------------------------------------------------------------------------>>>>
  //------------------------------------------------------------------------>>>>
  //------------------------------------------------------------------------>>>>
    
  }

void loop() 
  { 
    if (tcs0.readFast()) {
    Serial.print("TCS0: "); Serial.println(tcs0.getColor());
  }
    
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
    //Serial.print("gyro.gyro('z') :");Serial.print(gyro.gyro('z')); Serial.println( "   " ); 
    delay(10);
  }