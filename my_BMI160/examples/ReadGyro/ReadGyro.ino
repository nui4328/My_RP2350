#include <my_BMI160.h>
my_BMI160 gyro; // สร้างอ็อบเจ็กต์ด้วยที่อยู่เริ่มต้น (0x69)
void setup() {
  Serail.begin(9600);
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

}

void loop() {
  Serial.print("gyro.gyro('z') :");Serial.print(gyro.gyro('z')); Serial.println( "   " ); 
    delay(10);

}
