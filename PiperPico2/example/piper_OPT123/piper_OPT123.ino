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
EncoderLibrarys   encoder1(20, 19, 21, 22);
EncoderLibraryss  encoder2(16, 17, 10, 18);

#include "my_TCS.h"
my_TCS tcs0(0x0000);
my_TCS tcs1(0x0040);


void setup() {
  Serial.begin(115200);
  robot.begin();
  Wire.begin();
  gyro.setWire(Wire);
  if (gyro.begin()) {
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

    Set_travel_distance_toturnGyro (130); //------>> ตั้งค่า เคลื่อนที่ออกจากเส้นก่อนหมุนตัว
    set_pid_turnL(2.2, 0.005, 0.15);   // PID สำหรับหมุนซ้าย
    set_pid_turnR(2.2, 0.005, 0.15);    // PID สำหรับหมุนขวาSerial.print("gyro.gyro('z') :");
    selectAndRunMode();         // รอเลือกโหมดที่นี่จนกว่าจะ run
    robot.loadCalibration() ;
    robot.playTone(3000, 300);
    //robot.motor('A', 40);
    //move_fw(50, 1.5, 30, "line");
    //turnGyro(80, -90);

}

void loop() {

  
 if(digitalRead(2) == 0)
    {
      gyro.resetAngles();
    }

 
  Serial.print(gyro.gyro('z'), 1);
  Serial.println(" °");
  delay(10);

  
/*
  for(int i=0; i<10; i++)
    {
      Serial.print(robot.adcMax(i));
      Serial.print(" | ");
    }*/
  //Serial.println(robot.knopRead());
 
  delay(10);
  
}