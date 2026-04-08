#include <ArrayPico2.h>
#include <EncoderLibrarys.h>
//#include <my_GYRO160s.h>

EncoderLibrarys encoder(19, 20, 21, 22);
ArrayPico2 robot;
//my_GYRO160 gyro;

void setup() {

  robot.begin();
  encoder.setupEncoder();
  encoder.resetEncoders();
/*
  if (!gyro.begin()) {
    Serial.println("GYRO init failed!");
  }
  gyro.resetAngles();
*/
  robot.run();  // แสดงเมนู รอปุ่ม

  robot.Motor(80,80);
}

void loop() {
  robot.displayVoltage();  // แสดงบน OLED ด้วย ratio ใหม่

  // ทดสอบ print ค่า raw + คำนวณ (ลบหรือ comment ออกได้หลังทดสอบเสร็จ)
  uint16_t adc = robot.adcRead(10);
  float raw = (adc / 4095.0f) * 3.3f;
  float volt = raw * VOLTAGE_DIVIDER_RATIO;
  
  Serial.print("ADC ch10: "); Serial.print(adc);
  Serial.print("  Raw: "); Serial.print(raw, 3);
  Serial.print("V  Voltage: "); Serial.print(volt, 2);
  Serial.println("V");

  delay(800);  // อัพเดททุก ~0.8 วินาที
}