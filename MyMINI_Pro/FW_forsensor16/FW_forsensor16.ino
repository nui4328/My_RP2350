#include <EEPROM.h>

// ---------- 74HC4067 ----------
const byte S0  = A3;
const byte S1  = A2;
const byte S2  = A1;
const byte S3  = A0;
const byte SIG = A7;

// ปุ่มกด: RX / D0 ต่อกับ GND
const byte BUTTON_PIN = 0;

// ---------- เซนเซอร์และ LED ----------
const byte SENSOR_COUNT = 16;
const byte LED_COUNT = 15;
const uint16_t EEPROM_MAGIC = 0x4067;
const unsigned long CALIBRATION_TIME_MS = 10000UL;

// Sensor 0-12 -> D1-D13, Sensor 13 -> A5
const byte ledPins[LED_COUNT] = {
  1, 2, 3, 4, 5, 6, 7,
  8, 9, 10, 11, 12, 13, A4, A5
};

struct CalibrationData {
  uint16_t magic;
  uint16_t minimum[SENSOR_COUNT];
  uint16_t maximum[SENSOR_COUNT];
};

CalibrationData calibration;

bool calibrating = false;
bool calibrationValid = false;
unsigned long calibrationStartedAt = 0;

// ---------- ป้องกันปุ่มเด้ง ----------
bool lastRawButton = HIGH;
bool stableButton = HIGH;
unsigned long buttonChangedAt = 0;
const unsigned long DEBOUNCE_MS = 30;

// ---------- MUX ----------
void selectMuxChannel(byte channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
}

int readSensor(byte channel) {
  selectMuxChannel(channel);
  delayMicroseconds(100);
  return analogRead(SIG);
}

// ---------- LED ----------
void turnOffLeds() {
  for (byte i = 0; i < LED_COUNT; i++) {
    digitalWrite(ledPins[i], LOW);
  }
}

void turnOnLeds() {
  for (byte i = 0; i < LED_COUNT; i++) {
    digitalWrite(ledPins[i], HIGH);
  }
}

void startupLedAnimation() {
  const unsigned int STEP_DELAY_MS = 25;

  for (byte round = 0; round < 3; round++) {
    // ซ้าย -> ขวา
    for (byte i = 0; i < LED_COUNT; i++) {
      digitalWrite(ledPins[i], HIGH);
      delay(STEP_DELAY_MS);
      digitalWrite(ledPins[i], LOW);
    }

    // ขวา -> ซ้าย
    for (int i = LED_COUNT - 1; i >= 0; i--) {
      digitalWrite(ledPins[i], HIGH);
      delay(STEP_DELAY_MS);
      digitalWrite(ledPins[i], LOW);
    }

 
  }

  turnOffLeds();
}

void calibrationDoneAnimation() {
  for (byte flash = 0; flash < 3; flash++) {
    turnOnLeds();
    delay(180);
    turnOffLeds();
    delay(180);
  }
}

// ---------- คาลิเบรต ----------
void startCalibration() {
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    calibration.minimum[i] = 1023;
    calibration.maximum[i] = 0;
  }

  calibrating = true;
  calibrationValid = false;
  calibrationStartedAt = millis();
}

void updateCalibration() {
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    int value = readSensor(i);

    if (value < calibration.minimum[i]) calibration.minimum[i] = value;
    if (value > calibration.maximum[i]) calibration.maximum[i] = value;
  }
}

void finishCalibration() {
  calibration.magic = EEPROM_MAGIC;
  EEPROM.put(0, calibration);

  calibrating = false;
  calibrationValid = true;

  calibrationDoneAnimation();
}

// ---------- แสดงสถานะ LED ----------
bool isSensorBelowMidpoint(byte channel) {
  int value = readSensor(channel);

  int midpoint =
    // (calibration.minimum[channel] + calibration.maximum[channel]) / 2;
    (((calibration.minimum[channel] + calibration.maximum[channel]) / 2) + calibration.minimum[channel])/2;

  return value < midpoint;
}

// void updateSensorLeds() {
//   for (byte channel = 0; channel < LED_COUNT; channel++) {
//     bool sensorIsLow = isSensorBelowMidpoint(channel);

//     // สลับ HIGH / LOW หาก LED เป็น Active-Low
//     digitalWrite(ledPins[channel], sensorIsLow ? HIGH : LOW);
//   }
// }
void updateSensorLeds() {
  for (byte channel = 0; channel < LED_COUNT; channel++) {
    bool sensorIsLow = isSensorBelowMidpoint(channel);

    // ค่าต่ำกว่าค่ากลาง = LED ดับ
    // ค่าเท่ากับหรือสูงกว่าค่ากลาง = LED ติด
    digitalWrite(ledPins[channel], sensorIsLow ? LOW : HIGH);
  }
}
// ---------- ปุ่ม ----------
void handleButton() {
  bool rawButton = digitalRead(BUTTON_PIN);

  if (rawButton != lastRawButton) {
    buttonChangedAt = millis();
    lastRawButton = rawButton;
  }

  if ((millis() - buttonChangedAt) >= DEBOUNCE_MS &&
      rawButton != stableButton) {
    stableButton = rawButton;

    // กดหนึ่งครั้งเพื่อเริ่มคาลิเบรต
    if (stableButton == LOW && !calibrating) {
      startCalibration();
    }
  }
}

// ---------- Arduino ----------
void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  for (byte i = 0; i < LED_COUNT; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  turnOffLeds();
  startupLedAnimation();

  delay(200);  // รอให้สัญญาณเซนเซอร์นิ่ง

  EEPROM.get(0, calibration);
  calibrationValid = (calibration.magic == EEPROM_MAGIC);
}

void loop() {
  handleButton();

  if (calibrating) {
    updateCalibration();

    // แสดงสถานะ LED แบบสดระหว่างคาลิเบรต
    updateSensorLeds();

    if (millis() - calibrationStartedAt >= CALIBRATION_TIME_MS) {
      finishCalibration();
    }
  } else if (calibrationValid) {
    updateSensorLeds();
  } else {
    turnOffLeds();
  }
}