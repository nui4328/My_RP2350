#include <PiperPico2.h>
#include "EncoderLibrarys.h"
#include "EncoderLibraryss.h"
#include <Wire.h>
#include <my_BNO085.h>

my_BNO085 imu(0x4B, Wire); 
PiperPico2 robot;
EncoderLibrarys   encoder1(20, 19, 21, 22);
EncoderLibraryss  encoder2(16, 17, 10, 18);



void setup() {
  Serial.begin(115200);
  robot.begin();
  imu.begin();

  // หรือเลือก Wire
  //Wire1.begin();
  // gyro.setWire(Wire);
  encoder1.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
  encoder1.resetEncoders();  //--------------------->> ฟังก์ชันรอก
  encoder2.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
  encoder2.resetEncoders();  //--------------------->> ฟังก์ชันรอก


  delay(1000);
  robot.playTone(3000, 70); delay(70);
  robot.playTone(3000, 70); delay(70);
  imu.calibrate();
  //robot.showVoltageUntilButton();  // หรืออะไรก็ได้
}

void loop() {
 if(digitalRead(2) == 0)
    {
      imu.resetAngles();
    }

 
  Serial.print(imu.gyro('z'), 1);
  Serial.println(" °");
  delay(10);
  
/*
  for(int i=0; i<10; i++)
    {
      Serial.print(robot.adcMax(i));
      Serial.print(" | ");
    }
  Serial.println(robot.knopRead());
 
  delay(10);
  */
}