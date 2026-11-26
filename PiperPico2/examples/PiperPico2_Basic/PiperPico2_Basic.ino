// แท็บหลัก: เริ่มบอร์ดและเขียนโปรแกรมหุ่นยนต์ที่นี่
// ตั้งค่าฮาร์ดแวร์ใน set_bord และดูตัวอย่างคำสั่งใน README
#include <PiperPico2.h>

PiperPico2 robot;
bool boardReady = false;

bool set_bord();
void ui_begin();
bool ui_update();
bool readGyro(float& yawDegrees, float& zDegreesPerSecond);
bool resetGyroYaw();
bool readGyroYaw(float& yawDegrees);

void setup() {
  Serial.begin(115200);
  boardReady = set_bord();
  if (!boardReady) {
    Serial.println("Board initialization failed; motor commands disabled in this sketch.");
    return;
  }
  Serial.println("PiperPico2 ready. See README and set_bord tabs.");

  // รอในเมนูจนกดสั้นเลือก RUN: บิ๊ปยาว แล้วออกจากลูปเมนู
  while (!ui_update()) delay(1);
  resetGyroYaw();
  // เขียนโปรแกรมหุ่นยนต์ที่ทำครั้งเดียวไว้ตรงนี้ เริ่มทันทีหลังเสียง RUN จบ
  // robot.motor('A', 60);
  // robot.motor('C', 60);
  if (!robot.motorOK()) {
    Serial.println("Motor C command failed. Check PCA9685/I2C.");
  }
}

void loop() {
  float yaw;
  if (boardReady && readGyroYaw(yaw)) {
    Serial.println(yaw);
  }
  delay(30);
}
