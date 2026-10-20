#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <my_BNO085.h>

// OLED: GP4 SDA, GP5 SCL. BNO085: GP26 SDA, GP27 SCL.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 36
#define SCREEN_ADDRESS 0x3C
#define BUTTON_RESET 2
#define BUTTON_MODE 15

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
my_BNO085 imu(0x4B, Wire1);
bool smoothDisplay = true;
float filteredYaw = 0.0f;
const float alpha = 0.85f;

float wrapHeading360(float angle) {
  float wrapped = fmodf(angle, 360.0f);
  if (wrapped < 0.0f) wrapped += 360.0f;
  return wrapped >= 360.0f ? 0.0f : wrapped;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(BUTTON_RESET, INPUT_PULLUP);
  pinMode(BUTTON_MODE, INPUT_PULLUP);

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  Wire1.setClock(100000);
  Wire1.setTimeout(25, true);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED not found"));
    for (;;) delay(500);
  }
  if (!imu.begin()) {
    Serial.println(F("BNO085 not ready"));
    imu.printDiagnostics(Serial);
    for (;;) delay(500);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("BNO085 ready"));
  display.println(F("GP2 reset yaw"));
  display.display();
}

void loop() {
  static uint32_t lastResetMs = 0;
  static uint32_t lastModeMs = 0;
  const uint32_t now = millis();
  if (digitalRead(BUTTON_RESET) == LOW && now - lastResetMs > 300) {
    if (imu.resetYaw()) filteredYaw = 0.0f;
    lastResetMs = now;
  }
  if (digitalRead(BUTTON_MODE) == LOW && now - lastModeMs > 300) {
    smoothDisplay = !smoothDisplay;
    lastModeMs = now;
  }

  imu.update();
  if (imu.angleReady()) {
    // Keep the example's right-positive display convention.
    const float yaw = wrapHeading360(-imu.yawRaw());
    float delta = yaw - filteredYaw;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    filteredYaw = wrapHeading360(filteredYaw + (1.0f - alpha) * delta);
    const float shown = smoothDisplay ? filteredYaw : yaw;
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(smoothDisplay ? F("Smooth") : F("Raw"));
    display.setCursor(70, 0);
    display.print(F("z:"));
    display.print(-imu.rate('z'), 0);
    display.setTextSize(2);
    display.setCursor(0, 10);
    display.print(shown, 1);
    display.cp437(true);
    display.write(248);
    display.display();
  }
  delay(10);
}
