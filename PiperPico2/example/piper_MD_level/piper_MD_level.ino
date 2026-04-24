#include <PiperPico2.h>
#include "EncoderLibrarys.h"
#include "EncoderLibraryss.h"
#include <Wire.h>
#include <my_BMI160.h>
my_BMI160 gyro; 
#include "MoveGyroPID.h"



bool Connected_gyro;
PiperPico2 robot;
EncoderLibrarys   encoder1(20, 19, 22, 21);
EncoderLibraryss  encoder2(16, 17, 10, 18);

#include "my_TCS.h"
my_TCS tcs0(0x0000);
my_TCS tcs1(0x0040);


void setup() {
  Serial.begin(115200);
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  gyro.setWire(Wire1);

  robot.begin();
  //Wire.begin();
  //gyro.setWire(Wire);
  if (!gyro.begin()) {
    Serial.println("GYRO init failed!");
    //while (true) delay(10);
  }
  else {
    Connected_gyro = true;
  }
 //gyro.begin();
  initOLED();  // แสดง splash 1 วินาที + init จอ

  // หรือเลือก Wire
  //Wire1.begin();
  // gyro.setWire(Wire);
  encoder1.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
  encoder1.resetEncoders();  //--------------------->> ฟังก์ชันรอก
  encoder2.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
  encoder2.resetEncoders();  //--------------------->> ฟังก์ชันรอก


  
  //robot.showVoltageUntilButton();  // หรืออะไรก็ได้

    Set_travel_distance_toturnGyro (10); //------>> ตั้งค่า เคลื่อนที่ออกจากเส้นก่อนหมุนตัว
    set_pid_turnL(1.7, 0.0026, 0.06);   // PID สำหรับหมุนซ้าย
    set_pid_turnR(1.6, 0.0026, 0.06);    // PID สำหรับหมุนขวาSerial.print("gyro.gyro('z') :");

    robot.setServo(0, 160); delay(200);
    selectAndRunMode();         // รอเลือกโหมดที่นี่จนกว่าจะ run
    robot.loadCalibration() ;
    robot.playTone(3000, 300);
    //robot.motor('A', 40);
    
/////--------------------------------------------------------------------->>>
    //move_fw(30, 1.2, 60, "none_line");
   
 


///--------------------------------------------------------------->>>


}

void loop() {

  //open_servo();
  for(int i=0; i<9; i++)
      {
        Serial.print(robot.adcRead(i)); 
        Serial.print("   "); 
      }
    Serial.println(""); 

  /*
 if(digitalRead(2) == 0)
    {
      gyro.resetAngles();
    }

 
 
 */
   //Serial.println(gyro.gyro('z'), 1);
  //Serial.println(" °");
  delay(10);
/*
  for(int i=0; i<10; i++)
    {
      Serial.print(robot.adcMax(i));
      Serial.print(" | ");
    }
  */
  //Serial.println(robot.knopRead());
   //Serial.println(encoder1.Poss_R());
  //delay(10);
  
}