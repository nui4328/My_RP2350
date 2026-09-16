#include <Pico2_MyMiniPro.h>

Pico2MyMiniPro robot;

// Retain Start/calibration handling without telemetry output.
class SilentStream : public Stream {
 public:
  size_t write(uint8_t) override { return 1; }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
};

void setup() {
  robot.begin();
  MyMiniPro::ForwardSettings settings;
  settings.centimetersPerSecondPerPercent = 1.6875;
  settings.maximumRunMs = 3000;

  // ===== Setup: ปรับโปรไฟล์ความเร็ว F_Line ตรงนี้ =====
  // เวลาไต่ความเร็วตอนออกตัว หน่วยมิลลิวินาที (ms); 0 = ปิดการไต่ความเร็ว
  settings.startupRampMs = 300;
  // ระยะก่อนถึงเป้าหมายที่เริ่มชะลอ หน่วยเซนติเมตร (cm); 0 = ปิดการชะลอ
  // ใช้เฉพาะช่วงวิ่งตามระยะทาง ไม่ใช้กับคำสั่งที่จบด้วยเซ็นเซอร์
  settings.distanceDecelerationCm = 5.0;
  // ความเร็วปลายทางเป็นเปอร์เซ็นต์ของความเร็วเป้าหมายแต่ละล้อ (1..100)
  // เช่น เป้าหมาย 50 และค่า 20 จะเหลือประมาณ 10 เมื่อวิ่งตรง ก่อนชดเชยแรงดัน
  // คงระดับนี้ใน 1/5 สุดท้ายของระยะชะลอ: ตั้ง 5 cm จะคงความเร็วต่ำใน 1 cm ท้าย
  settings.distanceEndSpeedPercent = 20;
  // พิจารณาแยกล้อ: SPL หรือ SPR ต่ำกว่า 25 ข้ามทั้งออกตัวและชะลอสำหรับล้อนั้น
  // ค่า 25 ขึ้นไปใช้โปรไฟล์; เช่น 24/50 ล้อซ้ายตอบสนองทันที ล้อขวาใช้โปรไฟล์
  // ส่งค่าชุดนี้ให้ F_Line ใช้งานจริง; แต่ละคำสั่งเริ่มสถานะออกตัวใหม่
  robot.setForwardSettings(settings);
  robot.setMuxSettleMicroseconds(100);
  robot.setSensorSmoothing(50);
  robot.setFLineSensorDebounceMs(0);
  robot.setFLineKd(0.021);
  robot.setFLineRecoveryKpThreshold(0.4);
  robot.setFLineApproachSpeed(15, 15); // Speed after distance while approaching/crossing the line.
  robot.setFLineFRMotors(100, -30);
  robot.setFLineFLMotors(-30, 100);

  SilentStream silentOutput;
  while (!robot.waitButton(silentOutput, 100)) {}
  robot.F_Line(60, 60, 0.85, 40, ns, 40);
}

void loop() {}
