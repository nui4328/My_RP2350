// อุปกรณ์เสริมของเมนูเดิม: เปลี่ยน 0 เป็น 1 เมื่อติดตั้งไลบรารี/อุปกรณ์แล้ว
// COLOR ใช้ my_TCS/SoftwareWire; GYRO เลือก BNO055 หรือ my_BMI160
#ifndef BOARD_ENABLE_COLOR
#define BOARD_ENABLE_COLOR 0 // ใช้ BNO055 ที่ 0x29 บน Wire โดยไม่เริ่ม TCS0
#endif
#ifndef BOARD_ENABLE_GYRO
#define BOARD_ENABLE_GYRO 1
#endif
#define BOARD_GYRO_BMI160 2
#define BOARD_GYRO_BNO055 1
#ifndef BOARD_GYRO_TYPE
#define BOARD_GYRO_TYPE BOARD_GYRO_BNO055
#endif
#if BOARD_ENABLE_COLOR
#include <my_TCS.h>
my_TCS uiTcs0(0x0000); // EEPROM จริง 0x0080..0x00A4 (รวม COLOR_CAL_START_ADDR)
my_TCS uiTcs1(0x0040); // EEPROM จริง 0x00C0..0x00E4 ไม่ทับ ADC 0x0000..0x0027
#endif
#if BOARD_ENABLE_GYRO && BOARD_GYRO_TYPE == BOARD_GYRO_BNO055
#include <BNO055.h>
BNO055 uiBno055;
const uint8_t BOARD_BNO055_ADDRESS = 0x29; // ตั้ง ADR ของโมดูลเป็น 0x29 และถอด TCS0 ออกจาก Wire
#elif BOARD_ENABLE_GYRO && BOARD_GYRO_TYPE == BOARD_GYRO_BMI160
#include <my_BMI160.h>
my_BMI160 uiGyro(0x69); // เปลี่ยนเป็น 0x68 หากบอร์ดของคุณใช้ address นี้
#elif BOARD_ENABLE_GYRO
#error "Unsupported BOARD_GYRO_TYPE"
#endif

// OLED ของไลบรารี: 128x32, address 0x3C ใช้ framebuffer เดียว ไม่ begin() ซ้ำ
const uint16_t BOARD_UI_REFRESH_MS = 150;
const uint16_t BOARD_UI_DEBOUNCE_MS = 30;
const uint16_t BOARD_UI_BACK_MS = 3000;
const uint16_t BOARD_KNOB_MAX = 2800; // ค่าสูงสุด ADC ของปุ่มหมุน ปรับตามบอร์ดจริง
const float BOARD_LOW_BATTERY_V = 10.0f; // ปรับตามแบตเตอรี่ ไม่ใช่ระดับเปอร์เซ็นต์
const bool BOARD_UI_BATTERY_BEEP = false;

// KLJ-7525-3627: ความถี่ตามสเปก 2731 Hz; เมโลดีอยู่ช่วง 2.1–3.1 kHz
// ปรับความถี่ให้เหมาะกับบัซเซอร์ ไม่ใช่การเพิ่มแรงดันหรือกระแส GPIO
// ทำนอง C-D-E-G-E-D-C สูง เล่นครั้งเดียวพร้อมหน้าต้อนรับ
const bool BOARD_BOOT_MELODY_ENABLED = true;
const uint16_t BOARD_RUN_TONE_HZ = 2731;
const uint16_t BOARD_BOOT_NOTE_GAP_MS = 20;
const uint16_t BOARD_BOOT_MELODY[][2] = { // {ความถี่ Hz, ระยะเวลา ms}; Hz=0 คือพัก
  {2093, 90}, {2349, 90}, {2637, 130}, {3136, 90},
  {2637, 130}, {2349, 90}, {2093, 240}
};

// แท็บตั้งค่าบอร์ด: เปลี่ยนค่าด้านบนให้ตรงกับฮาร์ดแวร์จริง
// set_bord() เรียกครั้งเดียวจาก setup() ในแท็บหลัก

// 1) เลือกไดรเวอร์แต่ละมอเตอร์ A, B, C, D
// LEGACY = ไดรเวอร์เดิม, DRV8874 = PH/EN โดย PMODE ต้องต่อ GND
// ตัวอย่างเปลี่ยน A: PiperPico2::MotorDriver::DRV8874
const PiperPico2::MotorDriver BOARD_MOTOR_DRIVERS[4] = {
  PiperPico2::MotorDriver::LEGACY, // A: GPIO PWM 8
  PiperPico2::MotorDriver::LEGACY, // B: GPIO PWM 9
  PiperPico2::MotorDriver::DRV8874, // C: GPIO PWM 7, PCA PH 2, nSLEEP 3
  PiperPico2::MotorDriver::LEGACY  // D: GPIO PWM 6
};
const bool BOARD_MOTOR_REVERSED[4] = {false, false, false, false};

// 2) ช่วงพัลส์ Servo รายช่อง หน่วยไมโครวินาที (us)
// ตัวเลข 1000–2000 เป็นค่าตัวอย่าง ต้องปรับตาม Servo และกลไกจริง
// การตั้งช่วงยังไม่ทำให้ Servo เคลื่อน; มุม 0–180 จะถูกแมปเข้าช่วงนี้
const uint16_t BOARD_SERVO_MIN_US[6] = {1000, 1000, 1000, 1000, 1000, 1000};
const uint16_t BOARD_SERVO_MAX_US[6] = {2000, 2000, 2000, 2000, 2000, 2000};
// ช่องเรียก 0,1,2,3,4,5 -> PCA9685 ช่อง 15,14,13,12,11,10

// 3) ลำดับขา Encoder A/B — สลับสองตัวเลขในแถวเพื่อกลับทิศการนับ
// เปลี่ยนได้เฉพาะลำดับของคู่ขาเดิม ห้ามนำขาจากคนละคู่มาใช้
const uint8_t BOARD_ENCODER_PINS[4][2] = {
  {19, 20}, // 0: enc1Left()  -> สลับกลับเป็น {20, 19}
  {21, 22}, // 1: enc1Right() -> กลับทิศเป็น {22, 21}
  {16, 17}, // 2: enc2Left()  -> กลับทิศเป็น {17, 16}
  {10, 18}  // 3: enc2Right() -> กลับทิศเป็น {18, 10}
};

// 4) ความถี่ I2C และ GPIO PWM ของ Motor (ไม่ใช่ความถี่ Servo)
const uint32_t BOARD_I2C_CLOCK_HZ = 100000; // ไลบรารียอมรับ 100000–400000
const uint32_t BOARD_MOTOR_PWM_HZ = 15000; // ตั้งในช่วง 1000–50000

bool set_bord() {
  for (uint8_t i = 0; i < 4; ++i) {
    if (!robot.setMotorDriver('A' + i, BOARD_MOTOR_DRIVERS[i], BOARD_MOTOR_REVERSED[i])) {
      Serial.println("Invalid motor configuration.");
      return false;
    }
  }
  for (uint8_t channel = 0; channel < 6; ++channel) {
    if (!robot.setServoPulseRangeUs(channel, BOARD_SERVO_MIN_US[channel], BOARD_SERVO_MAX_US[channel])) {
      Serial.print("Invalid Servo pulse range on channel ");
      Serial.println(channel);
      return false;
    }
  }

  for (uint8_t i = 0; i < 4; ++i) {
    if (!robot.setEncoderPins(i, BOARD_ENCODER_PINS[i][0], BOARD_ENCODER_PINS[i][1])) {
      Serial.print("Invalid Encoder pin pair at index ");
      Serial.println(i);
      return false;
    }
  }

  // begin() เริ่ม Encoder ภายใน และปิด PWM Motor / พัลส์ Servo ตอนเริ่มต้น
  if (!robot.begin()) {
    robot.stopMotors();
    Serial.print("PWM ready: "); Serial.println(robot.pwmReady());
    Serial.print("OLED ready: "); Serial.println(robot.displayReady());
    if (robot.displayReady()) {
      auto& display = robot.getDisplay();
      display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0); display.print("BOARD INIT FAILED");
      display.setCursor(0, 8); display.print("PWM: "); display.print(robot.pwmReady() ? "OK" : "ERROR");
      display.setCursor(0, 24); display.print("Check power / I2C");
      display.display();
    }
    return false;
  }
  if (!robot.setI2CClock(BOARD_I2C_CLOCK_HZ)) {
    robot.stopMotors();
    Serial.println("Invalid I2C clock.");
    return false;
  }
  if (BOARD_MOTOR_PWM_HZ < 1000 || BOARD_MOTOR_PWM_HZ > 50000) {
    robot.stopMotors();
    Serial.println("Invalid motor PWM frequency.");
    return false;
  }
  robot.setMotorPWMFrequency(BOARD_MOTOR_PWM_HZ);
  robot.stopMotors();
  if (!robot.motorOK()) {
    Serial.println("Motor initialization failed.");
    return false;
  }
  robot.resetEncoders();

  // begin() โหลดคาลิเบรต Sensor ให้แล้ว; ไม่มีข้อมูลยังอ่านค่าดิบได้
  if (!robot.calibrationReady()) {
    Serial.println("Line calibration not loaded; default range is 0..4095.");
  }
  ui_begin();
  return true;
}

// ---------------- OLED UI: ใช้ knob เลือก, กดสั้นเข้า, ค้าง 3 วินาทีย้อนกลับ ----------------
enum UiPage : uint8_t {
  UI_MAIN, UI_READ, UI_ADC_MENU, UI_ADC_VIEW, UI_COLOR, UI_GYRO, UI_ENCODER,
  UI_CAL_MENU, UI_TCS_MENU, UI_ADC_CAL, UI_COLOR_CAL, UI_RUN, UI_MESSAGE
};
uint8_t uiPage = UI_MAIN, uiSelected = 0, uiAdcMode = 0;
uint8_t uiMessageReturn = UI_MAIN, uiColorSensor = 0, uiColorStep = 0;
uint32_t uiLastDraw = 0, uiLastBattery = 0, uiLastWarning = 0;
uint32_t uiRawChanged = 0, uiPressedAt = 0, uiMessageAt = 0, uiAdcCalAt = 0;
uint32_t uiSplashAt = 0;
bool uiSplashActive = false;
bool uiBootMelodyActive = false;
uint8_t uiBootNote = 0;
uint32_t uiBootNoteAt = 0;
bool uiRawPressed = false, uiStablePressed = false, uiHoldHandled = false;
bool uiColorReady[2] = {false, false}, uiGyroReady = false;
bool uiAddressConflict = false;
bool uiDirty = true;
bool uiColorRetry = false;
const char* uiMessageTitle = "";
const char* uiMessageDetail = "";
const char* uiMainItems[] = {"RUN", "READSENSOR", "CALIBRATESENSOR"};
const char* uiReadItems[] = {"ADC", "COLOR", "GYRO", "ENCODER"};
const char* uiAdcItems[] = {"adcRead", "adcMD", "adcMax", "adcMin"};
const char* uiCalItems[] = {"ADC-Cal", "TCS-Cal"};
const char* uiTcsItems[] = {"TCS0-Cal", "TCS1-Cal"};
#if BOARD_ENABLE_COLOR
const char* uiColorNames[] = {"RED", "GREEN", "BLUE", "YELLOW", "WHITE", "BLACK"};
#endif

void uiSetPage(uint8_t page) {
  uiPage = page;
  uiSelected = 0;
  uiDirty = true;
}

void uiMessage(const char* title, const char* detail, uint8_t returnPage) {
  uiMessageTitle = title;
  uiMessageDetail = detail;
  uiMessageReturn = returnPage;
  uiMessageAt = millis();
  uiSetPage(UI_MESSAGE);
}

void uiFlush() {
  robot.getDisplay().display();
  robot.setI2CClock(BOARD_I2C_CLOCK_HZ);
}

void uiSplash() {
  auto& display = robot.getDisplay();
  display.setTextWrap(false);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(34, 0); display.print("PIPER");
  display.setTextSize(1);
  display.setCursor(40, 20); display.print("ROBOTECH");
  uiFlush();
}

void uiBootMelodyUpdate() {
  if (!uiBootMelodyActive) return;
  const uint32_t now = millis();
  if (now - uiBootNoteAt < uint32_t(BOARD_BOOT_MELODY[uiBootNote][1]) + BOARD_BOOT_NOTE_GAP_MS) return;
  ++uiBootNote;
  if (uiBootNote >= sizeof(BOARD_BOOT_MELODY) / sizeof(BOARD_BOOT_MELODY[0])) {
    uiBootMelodyActive = false;
    return;
  }
  uiBootNoteAt = now;
  robot.playToneAsync(BOARD_BOOT_MELODY[uiBootNote][0], BOARD_BOOT_MELODY[uiBootNote][1]);
}

void ui_begin() {
  // ใช้จอที่ robot.begin() เริ่มแล้ว ไม่สร้าง display/Encoder หรือเริ่ม Wire ซ้ำ
  uiSplash();
#if BOARD_ENABLE_COLOR && BOARD_ENABLE_GYRO && BOARD_GYRO_TYPE == BOARD_GYRO_BNO055
  uiAddressConflict = BOARD_BNO055_ADDRESS == TCS34725_ADDRESS;
#else
  uiAddressConflict = false;
#endif
#if BOARD_ENABLE_GYRO
  auto& display = robot.getDisplay();
  display.clearDisplay(); display.setCursor(0, 0);
  display.print("GYRO: keep still"); uiFlush();
#if BOARD_GYRO_TYPE == BOARD_GYRO_BNO055
  uiGyroReady = !uiAddressConflict && uiBno055.begin(BOARD_BNO055_ADDRESS, Wire, -1, -1, BOARD_I2C_CLOCK_HZ);
  if (uiGyroReady) uiBno055.loadStoredGyroCalibration(); // อ่านค่าเดิมเท่านั้น ไม่สั่งคาลิเบรต/เขียน EEPROM
#else
  uiGyro.setWire(Wire);
  uiGyroReady = uiGyro.begin(); // driver มีการเก็บ offset ช่วงเริ่มต้นแบบบล็อก
#endif
#endif
#if BOARD_ENABLE_COLOR
  Wire1.setSDA(26); Wire1.setSCL(27); Wire1.begin();
  uiColorReady[0] = !uiAddressConflict && uiTcs0.begin(TCS34725_ADDRESS, &Wire);
  uiColorReady[1] = uiTcs1.begin(TCS34725_ADDRESS, &Wire1);
  if (uiColorReady[0]) uiColorReady[0] = uiTcs0.setConfig(TCS_IT_101MS, TCS_GAIN_16X);
  if (uiColorReady[1]) uiColorReady[1] = uiTcs1.setConfig(TCS_IT_101MS, TCS_GAIN_16X);
#endif
  robot.setI2CClock(BOARD_I2C_CLOCK_HZ); // คืนค่าหลัง driver เสริมเริ่มต้น
  robot.updateBattery();
  uiLastBattery = millis();
  uiRawPressed = digitalRead(PiperPico2::BUTTON_PIN) == LOW;
  uiStablePressed = uiRawPressed;
  uiRawChanged = uiPressedAt = millis();
  uiHoldHandled = uiRawPressed; // ปุ่มที่กดค้างตอนเปิดเครื่องต้องปล่อยก่อน
  uiSetPage(UI_MAIN);
  uiSplash();
  uiSplashAt = millis();
  uiSplashActive = true;
  uiBootNote = 0;
  uiBootNoteAt = uiSplashAt;
  uiBootMelodyActive = BOARD_BOOT_MELODY_ENABLED;
  if (uiBootMelodyActive) {
    robot.playToneAsync(BOARD_BOOT_MELODY[0][0], BOARD_BOOT_MELODY[0][1]);
  }
  if (uiAddressConflict) {
    Serial.println("I2C conflict: BNO055 and TCS0 both use 0x29 on Wire. Set BNO055 ADR and code to 0x28.");
    uiMessage("ADDR CONFLICT 0x29", "BNO055 / TCS0", UI_MAIN);
  }
}

uint8_t uiMenuCount() {
  switch (uiPage) {
    case UI_MAIN: return 3;
    case UI_READ: case UI_ADC_MENU: return 4;
    case UI_CAL_MENU: case UI_TCS_MENU: return 2;
    default: return 0;
  }
}

uint8_t uiKnobSelection(uint8_t count) {
  if (count == 0 || BOARD_KNOB_MAX == 0) return 0;
  const uint16_t raw = constrain(int(robot.adcRead(10)), 0, int(BOARD_KNOB_MAX));
  const uint32_t full = uint32_t(BOARD_KNOB_MAX) + 1;
  // Hysteresis 1% ของช่วง knob กันรายการกระโดดเมื่ออยู่ขอบเขต
  const uint16_t margin = BOARD_KNOB_MAX / 100;
  if (uiSelected < count) {
    const int32_t lower = int32_t(uint32_t(uiSelected) * full / count) - margin;
    const int32_t upper = int32_t(uint32_t(uiSelected + 1) * full / count) + margin;
    if (raw >= lower && raw < upper) return uiSelected;
  }
  return uint8_t(uint32_t(raw) * count / full);
}

void uiRestoreSelectedColor() {
#if BOARD_ENABLE_COLOR
  if (!uiColorReady[uiColorSensor]) return;
  if (uiColorSensor == 0) uiTcs0.loadColorCalibration();
  else uiTcs1.loadColorCalibration();
#endif
}

void uiBack() {
  if (uiPage == UI_RUN) return;
  robot.stopMotors();
  switch (uiPage) {
    case UI_ADC_VIEW: uiSetPage(UI_ADC_MENU); break;
    case UI_ADC_MENU: case UI_COLOR: case UI_GYRO: case UI_ENCODER: uiSetPage(UI_READ); break;
    case UI_ADC_CAL:
      robot.cancelCalibration(); uiSetPage(UI_CAL_MENU); break;
    case UI_TCS_MENU: uiSetPage(UI_CAL_MENU); break;
    case UI_COLOR_CAL:
      uiRestoreSelectedColor(); uiSetPage(UI_TCS_MENU); break;
    case UI_MESSAGE: uiSetPage(uiMessageReturn); break;
    default: uiSetPage(UI_MAIN); break;
  }
  robot.playToneAsync(800, 100);
}

void uiCaptureColor() {
#if BOARD_ENABLE_COLOR
  my_TCS& sensor = uiColorSensor == 0 ? uiTcs0 : uiTcs1;
  if (!sensor.readFast() || sensor.c <= 10) {
    uiColorRetry = true; uiDirty = true;
    robot.playToneAsync(600, 120);
    return; // ไม่มีข้อมูลใหม่ ให้กดเก็บสีเดิมอีกครั้ง
  }
  uiColorRetry = false;
  switch (uiColorStep) {
    case 0: sensor.calibrateRed(); break;
    case 1: sensor.calibrateGreen(); break;
    case 2: sensor.calibrateBlue(); break;
    case 3: sensor.calibrateYellow(); break;
    case 4: sensor.calibrateWhite(); break;
    case 5: sensor.calibrateBlack(); break;
  }
  if (++uiColorStep < 6) { uiDirty = true; return; }
  if (!sensor.allColorsCalibrated()) {
    uiRestoreSelectedColor();
    uiMessage("Capture failed", "Restart TCS-Cal", UI_TCS_MENU);
    return;
  }
  sensor.calibrateDone();
  const bool saved = sensor.lastCalibrationSaveSucceeded();
  uiMessage(saved ? (uiColorSensor == 0 ? "TCS0 saved" : "TCS1 saved") : "TCS save failed",
            saved ? "Calibration complete" : "Check EEPROM", UI_TCS_MENU);
#endif
}

void uiDraw();

void uiEnter() {
  if (uiPage == UI_MAIN && uiSelected == 0) {
    uiSetPage(UI_RUN);
    uiDraw();
    robot.playTone(BOARD_RUN_TONE_HZ, 500); // บิ๊ปยาวให้จบก่อนออกเมนูเข้าโปรแกรมหุ่นยนต์
    return;
  }
  robot.playToneAsync(2500, 50);
  switch (uiPage) {
    case UI_MAIN:
      robot.stopMotors();
      if (uiSelected == 1) uiSetPage(UI_READ);
      else uiSetPage(UI_CAL_MENU);
      break;
    case UI_READ:
      if (uiSelected == 0) uiSetPage(UI_ADC_MENU);
      else if (uiSelected == 1) uiSetPage(UI_COLOR);
      else uiSetPage(uiSelected == 2 ? UI_GYRO : UI_ENCODER);
      break;
    case UI_ADC_MENU:
      uiAdcMode = uiSelected; uiSetPage(UI_ADC_VIEW); break;
    case UI_CAL_MENU:
      robot.stopMotors();
      if (uiSelected == 0) {
        if (robot.startCalibration(10000)) {
          uiAdcCalAt = millis(); uiSetPage(UI_ADC_CAL);
        } else uiMessage("ADC-Cal failed", "Cannot start", UI_CAL_MENU);
      } else {
        uiSetPage(UI_TCS_MENU);
      }
      break;
    case UI_TCS_MENU:
#if BOARD_ENABLE_COLOR
        uiColorSensor = uiSelected;
        if (uiColorSensor == 0 && uiAddressConflict) {
          uiMessage("ADDR CONFLICT 0x29", "BNO055 / TCS0", UI_TCS_MENU); break;
        }
        if (!uiColorReady[uiColorSensor]) {
          uiMessage(uiColorSensor == 0 ? "TCS0 not found" : "TCS1 not found",
                    "Check wiring", UI_TCS_MENU); break;
        }
        if (uiColorSensor == 0) uiTcs0.startColorCalibration();
        else uiTcs1.startColorCalibration();
        uiColorStep = 0; uiColorRetry = false;
        uiSetPage(UI_COLOR_CAL);
#else
        uiMessage("TCS disabled", "Enable in set_bord", UI_TCS_MENU);
#endif
      break;
    case UI_COLOR_CAL: uiCaptureColor(); break;
    default: break;
  }
}

void uiDrawMenu(const char* title, const char* const* items, uint8_t count) {
  auto& display = robot.getDisplay();
  display.setCursor(0, 0); display.print(title);
  // จอเหลือ 3 แถวใต้หัวข้อ: เลื่อนหน้าต่างสำหรับเมนูที่มี 4 รายการ
  const uint8_t first = uiSelected >= 3 ? uiSelected - 2 : 0;
  for (uint8_t row = 0; row < 3 && first + row < count; ++row) {
    const uint8_t item = first + row;
    const uint8_t y = 8 + row * 8;
    const bool selected = item == uiSelected;
    if (selected) display.fillRect(0, y, 128, 8, SSD1306_WHITE);
    display.setTextColor(selected ? SSD1306_BLACK : SSD1306_WHITE);
    display.setCursor(2, y); display.print(items[item]);
    display.setTextColor(SSD1306_WHITE);
  }
}

void uiDrawEncoderCell(uint8_t x, uint8_t y, const char* label, long count) {
  auto& display = robot.getDisplay();
  display.setCursor(x, y); display.print(label);
  // Each 64-pixel cell fits a 3-character label plus 7 value characters.
  if (count >= -999999L && count <= 9999999L) display.print(count);
  else {
    const double millions = double(count) / 1000000.0;
    display.printf(millions > -999.0 && millions < 999.0 ? "%.1fM" : "%.0fM", millions);
  }
}

void uiDraw() {
  auto& display = robot.getDisplay();
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  switch (uiPage) {
    case UI_MAIN: {
      uiDrawMenu("MODE", uiMainItems, 3);
      display.setCursor(42, 0); display.print(robot.getBatteryVoltage(), 1); display.print("V");
      display.setCursor(88, 0);
      const float voltage = robot.getBatteryVoltage();
      display.print(voltage < 2.0f ? "NO BAT" : voltage < BOARD_LOW_BATTERY_V ? "LOW" : "OK");
      break;
    }
    case UI_READ: uiDrawMenu("READ SENSOR", uiReadItems, 4); break;
    case UI_ADC_MENU: uiDrawMenu("ADC", uiAdcItems, 4); break;
    case UI_CAL_MENU: uiDrawMenu("CALIBRATE SENSOR", uiCalItems, 2); break;
    case UI_TCS_MENU: uiDrawMenu("CALIBRATE TCS", uiTcsItems, 2); break;
    case UI_ADC_VIEW:
      display.setCursor(0, 0); display.print(uiAdcItems[uiAdcMode]);
      display.setCursor(60, 0); display.print("Hold3s back");
      for (uint8_t channel = 0; channel < 10; ++channel) {
        uint16_t value = 0;
        switch (uiAdcMode) {
          case 0: value = robot.readLine(channel); break;
          case 1: value = robot.adcMD(channel); break;
          case 2: value = robot.adcMax(channel); break;
          case 3: value = robot.adcMin(channel); break;
        }
        display.setCursor((channel % 4) * 32, 8 + (channel / 4) * 8);
        display.printf("%4u", unsigned(value));
      }
      break;
    case UI_ENCODER:
      display.setCursor(0, 0); display.print("ENCODER");
      uiDrawEncoderCell(0, 8, "1L:", robot.enc1Left());
      uiDrawEncoderCell(64, 8, "1R:", robot.enc1Right());
      uiDrawEncoderCell(0, 16, "2L:", robot.enc2Left());
      uiDrawEncoderCell(64, 16, "2R:", robot.enc2Right());
      display.setCursor(0, 24); display.print("Hold3s back");
      break;
    case UI_COLOR:
#if BOARD_ENABLE_COLOR
      for (uint8_t i = 0; i < 2; ++i) {
        display.setCursor(0, i * 8); display.print(i == 0 ? "T0: " : "T1: ");
        my_TCS& sensor = i == 0 ? uiTcs0 : uiTcs1;
        if (i == 0 && uiAddressConflict) display.print("ADDR CONFLICT");
        else if (!uiColorReady[i]) display.print("NOT FOUND");
        else if (!sensor.readFast()) display.print("READ ERROR");
        else display.print(sensor.getColor());
      }
#else
      display.setCursor(0, 0); display.print("COLOR disabled");
      display.setCursor(0, 8); display.print("Enable in set_bord");
#endif
      display.setCursor(0, 24); display.print("Hold3s back"); break;
    case UI_GYRO:
#if BOARD_ENABLE_GYRO
      display.setCursor(0, 0);
      if (uiAddressConflict) display.print("ADDR CONFLICT 0x29");
      else if (!uiGyroReady) display.print("GYRO not found");
#if BOARD_GYRO_TYPE == BOARD_GYRO_BNO055
      else if (!uiBno055.update()) {
        display.print("BNO055 READ ERROR");
        display.setCursor(0, 8);
        display.print(uiBno055.lastUpdateError() == BNO055UpdateError::ImplausibleHeading
                      ? "HEADING REJECTED" : "Check I2C wiring");
      } else {
        display.print("BNO055 SGAM:");
        BNO055CalibrationStatus status;
        if (uiBno055.calibrationStatus(status)) {
          display.print(status.system); display.print(status.gyro);
          display.print(status.accel); display.print(status.mag);
        } else display.print("????");
        display.setCursor(0, 8);
        const float yaw = uiBno055.yaw();
        if (yaw >= -99999.9f && yaw <= 99999.9f) display.printf("Y:%8.1f deg", double(yaw));
        else display.print("Y: out of range");
        display.setCursor(0, 16);
        display.printf("Z:%8.1f deg/s", double(uiBno055.gyro('z')));
      }
#else
      else { display.print("BMI160 Z: "); display.print(uiGyro.gyro('z'), 1); display.print(" deg"); }
#endif
#else
      display.setCursor(0, 0); display.print("GYRO disabled");
      display.setCursor(0, 8); display.print("Enable in set_bord");
#endif
      display.setCursor(0, 24); display.print("Hold3s back"); break;
    case UI_ADC_CAL: {
      display.setCursor(0, 0); display.print("ADC calibrating...");
      const uint32_t elapsed = millis() - uiAdcCalAt;
      const uint32_t remaining = elapsed < 10000 ? (10000 - elapsed + 999) / 1000 : 0;
      display.setCursor(0, 8); display.print(remaining); display.print("s: move over line");
      display.setCursor(0, 24); display.print("Hold3s cancel"); break;
    }
    case UI_COLOR_CAL:
#if BOARD_ENABLE_COLOR
      display.setCursor(0, 0); display.print("Cal T"); display.print(uiColorSensor);
      display.print(" "); display.print(uiColorNames[uiColorStep]);
      display.setCursor(0, 8); display.print(uiColorRetry ? "Read/light error" : "Place color, click");
      display.setCursor(0, 16); display.print(uiColorRetry ? "Short click to retry" : "Short click: capture");
      display.setCursor(0, 24); display.print("Hold3s cancel");
#endif
      break;
    case UI_RUN:
      display.setCursor(0, 0); display.print("RUNNING");
      break;
    case UI_MESSAGE:
      display.setCursor(0, 0); display.print(uiMessageTitle);
      display.setCursor(0, 16); display.print(uiMessageDetail); break;
  }
  if (uiStablePressed && !uiHoldHandled) {
    const uint32_t held = millis() - uiPressedAt;
    if (held >= 1500 && held < BOARD_UI_BACK_MS) {
      display.fillRect(0, 24, 128, 8, SSD1306_BLACK);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 24); display.print("Hold ");
      display.print((BOARD_UI_BACK_MS - held + 999) / 1000);
      display.print("s back");
    }
  }
  uiFlush();
}

bool resetGyroYaw() {
#if BOARD_ENABLE_GYRO && BOARD_GYRO_TYPE == BOARD_GYRO_BNO055
  return uiGyroReady && uiBno055.resetAngles();
#else
  return false;
#endif
}

bool readGyroYaw(float& yawDegrees) {
#if BOARD_ENABLE_GYRO && BOARD_GYRO_TYPE == BOARD_GYRO_BNO055
  if (!uiGyroReady || !uiBno055.update()) return false;
  yawDegrees = uiBno055.yaw();
  return true;
#else
  (void)yawDegrees;
  return false;
#endif
}

bool readGyro(float& yawDegrees, float& zDegreesPerSecond) {
#if BOARD_ENABLE_GYRO && BOARD_GYRO_TYPE == BOARD_GYRO_BNO055
  if (!readGyroYaw(yawDegrees)) return false;
  zDegreesPerSecond = uiBno055.gyro('z');
  return true;
#else
  (void)yawDegrees;
  (void)zDegreesPerSecond;
  return false;
#endif
}

bool ui_update() {
  if (uiPage == UI_RUN) return true; // RUN ส่งต่อให้โปรแกรมผู้ใช้ถาวร
  const uint32_t now = millis();
  const bool pressed = digitalRead(PiperPico2::BUTTON_PIN) == LOW;
  if (uiSplashActive) {
    uiBootMelodyUpdate();
    if (now - uiSplashAt < 1000 || uiBootMelodyActive) return false;
    uiSplashActive = false;
    uiRawPressed = uiStablePressed = pressed;
    uiRawChanged = uiPressedAt = now;
    uiHoldHandled = pressed;
    uiDirty = true;
    if (uiPage == UI_MESSAGE) uiMessageAt = now;
  }
  if (pressed != uiRawPressed) { uiRawPressed = pressed; uiRawChanged = now; }
  if (uiRawPressed != uiStablePressed && now - uiRawChanged >= BOARD_UI_DEBOUNCE_MS) {
    uiStablePressed = uiRawPressed;
    if (uiStablePressed) { uiPressedAt = now; uiHoldHandled = false; }
    else if (!uiHoldHandled) {
      if (now - uiPressedAt >= BOARD_UI_BACK_MS) uiBack();
      else uiEnter();
    }
  }
  if (uiPage == UI_RUN) return true;
  if (uiStablePressed && !uiHoldHandled && now - uiPressedAt >= BOARD_UI_BACK_MS) {
    uiHoldHandled = true; uiBack(); // ทำครั้งเดียวต่อการกดค้าง ไม่กดเข้าใหม่เมื่อปล่อย
  }
  const uint8_t count = uiMenuCount();
  if (count && !uiStablePressed && !uiRawPressed) {
    const uint8_t selection = uiKnobSelection(count);
    if (selection != uiSelected) { uiSelected = selection; uiDirty = true; }
  }
  if (uiPage == UI_ADC_CAL) {
    robot.updateCalibration();
    if (!robot.isCalibrating()) {
      const bool saved = robot.calibrationSaved();
      uiMessage(saved ? "ADC saved" : "ADC save failed",
                saved ? "Calibration complete" : "Check sensor/EEPROM", UI_CAL_MENU);
    }
  }
  if (uiPage == UI_MESSAGE && millis() - uiMessageAt >= 2000) uiSetPage(uiMessageReturn);
  if (now - uiLastBattery >= 1000) {
    uiLastBattery = now; robot.updateBattery();
    const float voltage = robot.getBatteryVoltage();
    if (BOARD_UI_BATTERY_BEEP && voltage >= 2.0f && voltage < BOARD_LOW_BATTERY_V &&
        now - uiLastWarning >= 10000) {
      robot.playToneAsync(2000, 100); uiLastWarning = now;
    }
  }
  if (uiDirty || now - uiLastDraw >= BOARD_UI_REFRESH_MS) {
    uiDirty = false; uiLastDraw = now; uiDraw();
  }
  return uiPage == UI_RUN;
}
