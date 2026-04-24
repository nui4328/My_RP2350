#include <PiperPico2.h>
#include "EncoderLibrarys.h"
#include "EncoderLibraryss.h"
#include <my_BMI160.h>
my_BMI160 gyro; // สร้างอ็อบเจ็กต์ด้วยที่อยู่เริ่มต้น (0x69)

PiperPico2 robot;
EncoderLibrarys   encoder1(20, 19, 21, 22);
EncoderLibraryss  encoder2(16, 17, 10, 18);


void setup() {
  Serial.begin(115200);
  robot.begin();
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  gyro.setWire(Wire1);

  // หรือเลือก Wire
  // Wire.begin();
  // gyro.setWire(Wire);

  if (!gyro.begin()) {
    Serial.println("GYRO init failed!");
    //while (true) delay(10);
  }

  gyro.resetAngles(); 
  encoder1.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
  encoder1.resetEncoders();  //--------------------->> ฟังก์ชันรอก
  encoder2.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
  encoder2.resetEncoders();  //--------------------->> ฟังก์ชันรอก

  delay(1000);
  robot.playTone(3000, 70); delay(70);
  robot.playTone(3000, 70); delay(70);
  robot.showVoltageUntilButton();  // หรืออะไรก็ได้
}

void loop() {

  Serial.print("gyro.gyro('z') :");Serial.print(gyro.gyro('z')); Serial.println( "   " ); 
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