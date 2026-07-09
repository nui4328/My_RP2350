#include <BNO055.h>

BNO055 imu;

void setup() {
      Serial.begin(115200);
  delay(100);

  if (!imu.begin()) {           // หรือระบุพิน SDA,SCL ถ้าต้องการ
    Serial.println("BNO055 ไม่ตอบสนอง!");
    //while (1) delay(10);
  }

    imu.update();
    imu.resetAngles();
}

void loop() {
    imu.update();
    Serial.println(imu.yaw());
    delay(10);
}
