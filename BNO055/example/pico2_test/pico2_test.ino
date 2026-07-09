#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>

// OLED: I2C0 GP4 SDA, GP5 SCL
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 36
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// BNO: I2C1 GP26 SDA, GP27 SCL
Adafruit_BNO08x bno08x;

// ปุ่ม
#define BUTTON_RESET  2
#define BUTTON_MODE   15

float yawOffset = 0.0f;
bool offsetSet = false;
bool showAbsolute = false;

float filteredYaw = 0.0f;
const float alpha = 0.85f;
const float deadzone = 0.5f;

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(BUTTON_RESET, INPUT_PULLUP);
  pinMode(BUTTON_MODE, INPUT_PULLUP);

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED ไม่ตอบสนอง!");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("เริ่มต้น...");
  display.display();
  delay(1500);

  if (!bno08x.begin_I2C(0x4B, &Wire)) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("BNO ไม่พบ!");
    display.display();
    while (1);
  }

  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("พร้อมใช้งาน");
  display.println("GP2: Reset 0");
  display.display();
  delay(2000);
}

void loop() {
  // Reset Yaw
  static unsigned long lastReset = 0;
  if (digitalRead(BUTTON_RESET) == LOW && millis() - lastReset > 300) {
    offsetSet = false;
    filteredYaw = 0.0f;
    Serial.println("Reset Yaw → 0");
    lastReset = millis();
    delay(50);
  }

  // สลับโหมด absolute / relative
  static unsigned long lastMode = 0;
  if (digitalRead(BUTTON_MODE) == LOW && millis() - lastMode > 300) {
    showAbsolute = !showAbsolute;
    Serial.print("โหมด: ");
    Serial.println(showAbsolute ? "Absolute" : "Relative");
    lastMode = millis();
    delay(50);
  }

  sh2_SensorValue_t sensor_value;
  if (bno08x.getSensorEvent(&sensor_value)) {
    if (sensor_value.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qw = sensor_value.un.gameRotationVector.real;
      float qx = sensor_value.un.gameRotationVector.i;
      float qy = sensor_value.un.gameRotationVector.j;
      float qz = sensor_value.un.gameRotationVector.k;

      // Normalize quaternion
      float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
      if (norm > 0.001f) {
        qw /= norm; qx /= norm; qy /= norm; qz /= norm;
      }

      // คำนวณ yaw จาก quaternion (สูตร Euler ZYX)
      float siny = 2.0f * (qw * qz + qx * qy);
      float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
      float yaw_rad = atan2(siny, cosy);

      // ปรับ sign ให้หมุนขวา = บวก
      yaw_rad = -yaw_rad;

      float yaw_deg = yaw_rad * 180.0f / PI;

      float rawYaw = showAbsolute ? yaw_deg : (yaw_deg - yawOffset);

      // ตั้ง offset ครั้งแรก
      if (!offsetSet && !showAbsolute) {
        yawOffset = yaw_deg;
        offsetSet = true;
      }

      // กรอง EMA
      filteredYaw = alpha * filteredYaw + (1.0f - alpha) * rawYaw;

      // Deadzone เพื่อลดการกระตุก
      static float lastShown = 0.0f;
      if (abs(filteredYaw - lastShown) < deadzone) {
        filteredYaw = lastShown;
      } else {
        lastShown = filteredYaw;
      }

      // แสดงผลบนจอ
      display.clearDisplay();

      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print(showAbsolute ? "Abs" : "Rel");

      display.setCursor(40, 0);
      display.print("Acc:");
      display.print(sensor_value.status);  // ใช้ status แทน accuracy

      display.setTextSize(2);
      display.setCursor(0, 10);
      display.print(filteredYaw, 1);
      display.print(" ");
      display.cp437(true);
      display.write(248);

      if (filteredYaw > 5.0) {
        display.setCursor(SCREEN_WIDTH - 24, 10);
        display.setTextSize(2);
        display.print("->");
      } else if (filteredYaw < -5.0) {
        display.setCursor(SCREEN_WIDTH - 24, 10);
        display.setTextSize(2);
        display.print("<-");
      }

      display.display();
    }
  }

  delay(10);
}