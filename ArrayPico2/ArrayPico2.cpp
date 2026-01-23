#include "ArrayPico2.h"
#include <Arduino.h>

// ------------------- Initialization -------------------
void ArrayPico2::begin() {
  Serial.begin(115200);
  delay(1800);

  Wire.begin();
  delay(50);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  analogReadResolution(12);
  analogWriteResolution(8);
  analogWriteFreq(motor_pwm_freq);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  showWelcomeScreen();

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  loadCalibrationFromEEPROM();
  stop();

  Serial.print("Motor PWM frequency initialized at: ");
  Serial.print(motor_pwm_freq);
  Serial.println(" Hz");
}

// PWM Frequency
void ArrayPico2::setMotorPWMFrequency(uint32_t freqHz) {
  freqHz = constrain(freqHz, 100, 1000000);
  motor_pwm_freq = freqHz;
  analogWriteFreq(freqHz);
  Serial.print("Motor PWM frequency set to: ");
  Serial.print(freqHz);
  Serial.println(" Hz");
  tone(BUZZER_PIN, 1800, 80);
}

uint32_t ArrayPico2::getMotorPWMFrequency() const {
  return motor_pwm_freq;
}

// Motor Control
void ArrayPico2::Motor(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, -100, 100);
  rightSpeed = constrain(rightSpeed, -100, 100);

  if (leftSpeed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, map(leftSpeed, 0, 100, 0, 255));
  } else if (leftSpeed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, map(-leftSpeed, 0, 100, 0, 255));
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }

  if (rightSpeed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, map(rightSpeed, 0, 100, 0, 255));
  } else if (rightSpeed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, map(-rightSpeed, 0, 100, 0, 255));
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}

void ArrayPico2::forward(int speed) { Motor(speed, speed); }
void ArrayPico2::backward(int speed) { Motor(-speed, -speed); }
void ArrayPico2::turnLeft(int speed) { Motor(-speed, speed); }
void ArrayPico2::turnRight(int speed) { Motor(speed, -speed); }
void ArrayPico2::stop() { Motor(0, 0); }

// Servo
void ArrayPico2::ServoAttach(uint8_t pin) {
  if (pin == 16) servo16.attach(16);
  else if (pin == 17) servo17.attach(17);
  else if (pin == 18) servo18.attach(18);
}

void ArrayPico2::ServoWrite(uint8_t pin, int degree) {
  degree = constrain(degree, 0, 180);
  if (pin == 16) {
    servo16.attach(16);
    servo16.write(degree);
  } else if (pin == 17) {
    servo17.attach(17);
    servo17.write(degree);
  } else if (pin == 18) {
    servo18.attach(18);
    servo18.write(degree);
  }
}

void ArrayPico2::ServoDetach(uint8_t pin) {
  if (pin == 16) servo16.detach();
  else if (pin == 17) servo17.detach();
  else if (pin == 18) servo18.detach();
}

// Sensor
uint16_t ArrayPico2::adcRead(uint8_t channel) {
  digitalWriteFast(MUX_S0, channel & 1);
  digitalWriteFast(MUX_S1, (channel >> 1) & 1);
  digitalWriteFast(MUX_S2, (channel >> 2) & 1);
  digitalWriteFast(MUX_S3, (channel >> 3) & 1);

  delayMicroseconds(3);

  uint32_t sum = 0;
  for (uint8_t i = 0; i < SAMPLES_PER_READ; i++) {
    sum += analogRead(MUX_Z);
    delayMicroseconds(1);
  }
  return sum / SAMPLES_PER_READ;
}

uint16_t ArrayPico2::adcMin(uint8_t channel) {
  return (channel < NUM_SENSORS) ? sensorMin[channel] : 0;
}

uint16_t ArrayPico2::adcMax(uint8_t channel) {
  return (channel < NUM_SENSORS) ? sensorMax[channel] : 4095;
}

uint16_t ArrayPico2::adcMD(uint8_t channel) {
  if (channel >= NUM_SENSORS || sensorMax[channel] <= sensorMin[channel]) return 2048;
  return (sensorMin[channel] + sensorMax[channel]) / 2;
}

// UI
void ArrayPico2::displayVoltage() {
  uint16_t adc_value = adcRead(10);
  float voltage = (adc_value / 4095.0f) * 3.3f * VOLTAGE_DIVIDER_RATIO;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 2);
  display.println("Battery Voltage");

  display.setTextSize(2);
  display.setCursor(15, 12);
  display.print(voltage, 2);
  display.print(" V");

  if (voltage < 7.0f) {
    display.setTextSize(1);
    display.setCursor(70, 24);
    display.print("LOW BAT!");
  }
  display.display();
}

void ArrayPico2::showWelcomeScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(40, 1);
  display.println("Welcome to");

  display.setTextSize(2);
  display.setCursor(8, 14);
  display.println("<-ARRAY!->");

  display.drawLine(0, 30, 127, 30, SSD1306_WHITE);
  display.display();

  playStartupSound();
  delay(100);
}

void ArrayPico2::showMainMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(25, 1);
  display.println("Array-robotech");

  display.drawLine(10, 12, 125, 12, SSD1306_WHITE);

  display.setCursor(15, 16);
  display.println("Short press = RUN");

  display.setCursor(15, 24);
  display.println("Hold 2s = SETTINGS");

  display.display();
}

void ArrayPico2::run() {
  showMainMenu();

  while (digitalRead(BUTTON_PIN) == LOW) delay(10);
  delay(300);

  waitForButtonAction();
}

void ArrayPico2::waitForButtonAction() {
  tone(BUZZER_PIN, 3000, 50); delay(200);
  tone(BUZZER_PIN, 3000, 50); delay(200);
  noTone(BUZZER_PIN);

  while (digitalRead(BUTTON_PIN) == LOW) delay(10);
  delay(300);

  uint32_t buttonPressTime = 0;

  while (true) {
    bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

    if (buttonPressed) {
      if (buttonPressTime == 0) buttonPressTime = millis();

      if (millis() - buttonPressTime >= 2000) {
        tone(BUZZER_PIN, 3000, 300);
        delay(400);
        noTone(BUZZER_PIN);
        enterSettingsMode();
        showMainMenu();
        buttonPressTime = 0;
        return;
      }
    } else {
      if (buttonPressTime != 0) {
        uint32_t duration = millis() - buttonPressTime;
        if (duration > 50 && duration < 2000) {
          tone(BUZZER_PIN, 3000, 500);
          delay(600);
          noTone(BUZZER_PIN);

          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(20, 8);
          display.println("RUNNING");
          display.display();

          Serial.println("Robot is RUNNING!");
          return;
        }
        buttonPressTime = 0;
      }
    }
    delay(30);
  }
}

void ArrayPico2::enterSettingsMode() {
  const char* menuItems[3] = {"Calibrate", "View Sensor", "Exit"};
  int currentItem = 0;
  uint32_t pressTime = 0;

  while (true) {
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);

    if (pressed) {
      if (pressTime == 0) pressTime = millis();

      if (millis() - pressTime >= 2000) {
        tone(BUZZER_PIN, 2000, 300);
        delay(400);
        noTone(BUZZER_PIN);

        if (currentItem == 0) {
          calibrateSensors();

          while (digitalRead(BUTTON_PIN) == LOW) delay(10);
          delay(400);
          pressTime = 0;

          return;
        } else if (currentItem == 1) {
          viewSensorValues();
          return;
        } else if (currentItem == 2) {
          display.clearDisplay();
          display.setTextSize(1);
          display.setCursor(30, 10);
          display.println("EXITING...");
          display.display();

          tone(BUZZER_PIN, 2500, 150); delay(200);
          tone(BUZZER_PIN, 3000, 150); delay(200);
          tone(BUZZER_PIN, 2500, 150); delay(200);
          tone(BUZZER_PIN, 3000, 150); delay(300);
          noTone(BUZZER_PIN);

          while (digitalRead(BUTTON_PIN) == LOW) delay(10);
          delay(200);

          showMainMenu();
          waitForButtonAction();
          return;
        }
      }
    } else {
      if (pressTime != 0) {
        uint32_t dur = millis() - pressTime;
        if (dur > 50 && dur < 2000) {
          currentItem = (currentItem + 1) % 3;
          tone(BUZZER_PIN, 2500, 100);
          delay(150);
        }
        pressTime = 0;
      }
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    for (int i = 0; i < 3; i++) {
      int y = 2 + i * 9;

      if (i == currentItem) {
        display.fillRect(0, y - 1, 128, 9, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(8, y);
        display.print("> ");
        display.print(menuItems[i]);
        display.setTextColor(SSD1306_WHITE);
      } else {
        display.setCursor(20, y);
        display.print(menuItems[i]);
      }
    }

    display.setCursor(5, 25);
    display.println("Short=Next Hold=Enter");

    display.display();
    delay(50);
  }
}

// calibrateSensors - ปรับ delay หลัง save + ล้างสถานะ
void ArrayPico2::calibrateSensors() {
    tone(BUZZER_PIN, 2500, 150); delay(200);
    tone(BUZZER_PIN, 3000, 150); delay(200);
    noTone(BUZZER_PIN);

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(15, 8);
    display.println("CALIBRATE");
    display.display();

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
    }

    uint32_t lastBeep = 0;
    uint8_t displayMode = 0;
    uint32_t lastSwitch = 0;

    while (true) 
    {
        for (int i = 0; i < NUM_SENSORS; i++) {
            sensorValues[i] = adcRead(i);
            if (sensorValues[i] < sensorMin[i]) sensorMin[i] = sensorValues[i];
            if (sensorValues[i] > sensorMax[i]) sensorMax[i] = sensorValues[i];
        }

        if (millis() - lastBeep > 1000) {
            tone(BUZZER_PIN, 1500, 100);
            lastBeep = millis();
        }

        if (millis() - lastSwitch > 1500) {
            displayMode = (displayMode + 1) % 3;
            lastSwitch = millis();
        }

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        if (displayMode == 0) display.println("Min Values:");
        else if (displayMode == 1) display.println("Max Values:");
        else display.println("Threshold:");

        for (int i = 0; i < 5; i++) {
            if (displayMode == 0) display.printf("%4d ", sensorMin[i]);
            else if (displayMode == 1) display.printf("%4d ", sensorMax[i]);
            else display.printf("%4d ", adcMD(i));
        }
        display.setCursor(0, 16);
        for (int i = 5; i < NUM_SENSORS; i++) {
            if (displayMode == 0) display.printf("%4d ", sensorMin[i]);
            else if (displayMode == 1) display.printf("%4d ", sensorMax[i]);
            else display.printf("%4d ", adcMD(i));
        }
        display.display();

        if (digitalRead(BUTTON_PIN) == LOW) {
            delay(300);
            if (digitalRead(BUTTON_PIN) == LOW) {
                saveCalibrationToEEPROM();

                display.clearDisplay();
                display.setTextSize(2);
                display.setCursor(35, 8);
                display.println("SAVED!");
                display.display();

                for (int k = 0; k < 6; k++) {
                    tone(BUZZER_PIN, 3000, 100);
                    delay(150);
                }
                noTone(BUZZER_PIN);

                delay(800);

                while (digitalRead(BUTTON_PIN) == LOW) delay(10);
                delay(300);

                break;
            }
        }
    }
    run(); 
}

// viewSensorValues
void ArrayPico2::viewSensorValues() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Raw Sensor Values");
  display.setCursor(0, 10);
  display.println("Press any to exit");
  display.display();
  delay(100);

  while (true) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      delay(200);
      if (digitalRead(BUTTON_PIN) == LOW) {
        tone(BUZZER_PIN, 2000, 100); delay(150);
        tone(BUZZER_PIN, 1500, 100);
        noTone(BUZZER_PIN);
        return;
      }
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = adcRead(i);
    }

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Raw Values:");
    for (int i = 0; i < 5; i++) display.printf("%4d ", sensorValues[i]);
    display.setCursor(0, 16);
    for (int i = 5; i < 10; i++) display.printf("%4d ", sensorValues[i]);
    display.display();

    delay(100);
  }
}

// EEPROM
void ArrayPico2::loadCalibrationFromEEPROM() {
  uint16_t addr = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = readWordFromEEPROM(addr);
    addr += 2;
    sensorMax[i] = readWordFromEEPROM(addr);
    addr += 2;
  }
  Serial.println("Loaded calibration from EEPROM");
}

void ArrayPico2::saveCalibrationToEEPROM() {
  uint16_t addr = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    writeWordToEEPROM(addr, sensorMin[i]);
    addr += 2;
    writeWordToEEPROM(addr, sensorMax[i]);
    addr += 2;
  }
  Serial.println("Calibration saved to EEPROM!");
}

void ArrayPico2::writeWordToEEPROM(uint16_t eeaddr, uint16_t data) {
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((eeaddr >> 8) & 0xFF);
  Wire.write(eeaddr & 0xFF);
  Wire.write(data & 0xFF);
  Wire.write((data >> 8) & 0xFF);
  Wire.endTransmission();
  delay(5);
}

uint16_t ArrayPico2::readWordFromEEPROM(uint16_t eeaddr) {
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((eeaddr >> 8) & 0xFF);
  Wire.write(eeaddr & 0xFF);
  Wire.endTransmission();

  Wire.requestFrom(EEPROM_ADDRESS, 2);

  uint16_t data = 0;
  if (Wire.available() >= 2) {
    data = Wire.read();
    data |= (Wire.read() << 8);
  }
  return data;
}

// Sound
void ArrayPico2::playStartupSound() {
  tone(BUZZER_PIN, 2200, 100); delay(120);
  tone(BUZZER_PIN, 2600, 100); delay(120);
  tone(BUZZER_PIN, 3000, 200); delay(300);
  noTone(BUZZER_PIN);
}