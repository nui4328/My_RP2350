#include "UIEsp_bordmini.h"

UIEsp_bordmini::UIEsp_bordmini(MMEsp_bordmini& boardRef, my_GYRO1601* gyroPtr, EncoderLibrary* encoderPtr)
  : _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET), 
    board(boardRef), 
    gyro(gyroPtr), 
    encoder(encoderPtr) {}

void UIEsp_bordmini::begin() {
  Wire.begin(15, 16);

  if (!_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED failed"));
    for (;;);
  }
  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setTextColor(SSD1306_WHITE);
  _display.display();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void UIEsp_bordmini::playTone(int freq, int dur) {
  ledcAttach(BUZZER_PIN, freq, 10);
  ledcWriteTone(BUZZER_PIN, freq);
  delay(dur);
  ledcWriteTone(BUZZER_PIN, 0);
  ledcDetach(BUZZER_PIN);
}

void UIEsp_bordmini::playStartupSound() {
  playTone(2200, 100); delay(120);
  playTone(2600, 100); delay(120);
  playTone(3000, 200); delay(300);
}

void UIEsp_bordmini::showWelcomeScreen() {
  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setCursor(20, 4);
  _display.println("Welcome to");
  _display.setTextSize(2);
  _display.setCursor(8, 14);
  _display.println("MYMAKERS!");
  _display.drawLine(0, 30, 127, 30, SSD1306_WHITE);
  _display.display();
  playStartupSound();
  delay(1500);
}

void UIEsp_bordmini::showMainMenu() {
  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setTextColor(SSD1306_WHITE);
  _display.setCursor(20, 4);
  _display.println("MYMAKERS Robot");
  _display.drawLine(10, 12, 118, 12, SSD1306_WHITE);
  _display.setCursor(10, 16);
  _display.println("Short = RUN");
  _display.setCursor(10, 24);
  _display.println("Hold 2s = SETTINGS");
  _display.display();
}

void UIEsp_bordmini::waitForButtonAction() {
  playTone(3000, 100); delay(100);
  playTone(3000, 100); delay(100);

  uint32_t pressTime = 0;
  while (true) {
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);
    if (pressed) {
      if (pressTime == 0) pressTime = millis();
      if (millis() - pressTime >= 2000) {
        playTone(2000, 300);
        delay(400);
        enterSettingsMode();
        showMainMenu();
        pressTime = 0;
      }
    } else {
      if (pressTime != 0) {
        uint32_t dur = millis() - pressTime;
        if (dur > 50 && dur < 2000) {
          playTone(3000, 500);
          delay(600);
          _display.clearDisplay();
          _display.setTextSize(2);
          _display.setCursor(15, 8);
          _display.println("RUNNING!");
          _display.display();
          Serial.println(">>> Robot RUNNING <<<");
          return;
        }
        pressTime = 0;
      }
    }
    delay(20);
  }
}

void UIEsp_bordmini::enterSettingsMode() {
  const char* menuItems[3] = {"Calibrate", "View Sensor", "Exit"};
  int currentItem = 0;
  uint32_t pressTime = 0;

  while (true) {
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);
    if (pressed) {
      if (pressTime == 0) pressTime = millis();
      if (millis() - pressTime >= 2000) {
        playTone(2000, 300);
        delay(400);
        if (currentItem == 0) {
          calibrateSensors();
          showMainMenu();
          waitForButtonAction();
          return;
        } else if (currentItem == 1) {
          enterSensorMode();
        } else if (currentItem == 2) {
          _display.clearDisplay();
          _display.setTextSize(1);
          _display.setCursor(30, 10);
          _display.println("EXITING...");
          _display.display();
          playTone(2500, 150); delay(200);
          playTone(3000, 150); delay(200);
          playTone(2500, 150); delay(200);
          playTone(3000, 150); delay(300);
          delay(500);
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
          playTone(2500, 100);
          delay(150);
        }
        pressTime = 0;
      }
    }

    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(SSD1306_WHITE);
    for (int i = 0; i < 3; i++) {
      int y = 4 + i * 8;
      if (i == currentItem) {
        _display.fillRect(0, y - 1, 128, 9, SSD1306_WHITE);
        _display.setTextColor(SSD1306_BLACK);
        _display.setCursor(8, y);
        _display.print("> ");
        _display.print(menuItems[i]);
        _display.setTextColor(SSD1306_WHITE);
      } else {
        _display.setCursor(20, y);
        _display.print(menuItems[i]);
      }
    }
    _display.setCursor(5, 27);
    _display.println("Short=Next Hold=OK");
    _display.display();
    delay(50);
  }
}

void UIEsp_bordmini::enterSensorMode() {
  const char* menuItems[3] = {"ADCSensor", "Encoder&Gyro&Color", "Exit"};
  int currentItem = 0;
  uint32_t pressTime = 0;

  while (true) {
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);
    if (pressed) {
      if (pressTime == 0) pressTime = millis();
      if (millis() - pressTime >= 2000) {
        playTone(2000, 300);
        delay(400);
        if (currentItem == 0) {
          viewSensorValues();
        } else if (currentItem == 1) {
          viewGyroValues();
        } else if (currentItem == 2) {
          _display.clearDisplay();
          _display.setTextSize(1);
          _display.setCursor(30, 10);
          _display.println("EXITING...");
          _display.display();
          playTone(2000, 300); delay(400);
          playTone(3000, 50); delay(50);
          playTone(3000, 50); delay(50);
          delay(500);
          delay(500);
          return;
        }
      }
    } else {
      if (pressTime != 0) {
        uint32_t dur = millis() - pressTime;
        if (dur > 50 && dur < 2000) {
          currentItem = (currentItem + 1) % 3;
          playTone(2500, 100);
          delay(150);
        }
        pressTime = 0;
      }
    }

    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(SSD1306_WHITE);
    for (int i = 0; i < 3; i++) {
      int y = 4 + i * 8;
      if (i == currentItem) {
        _display.fillRect(0, y - 1, 128, 9, SSD1306_WHITE);
        _display.setTextColor(SSD1306_BLACK);
        _display.setCursor(8, y);
        _display.print("> ");
        _display.print(menuItems[i]);
        _display.setTextColor(SSD1306_WHITE);
      } else {
        _display.setCursor(20, y);
        _display.print(menuItems[i]);
      }
    }
    _display.setCursor(5, 27);
    _display.println("Short=Next Hold=OK");
    _display.display();
    delay(50);
  }
}

void UIEsp_bordmini::viewSensorValues() {
  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setCursor(5, 10);
  _display.println("Raw Sensor Values");
  _display.setCursor(5, 25);
  _display.println("Hold 2s = Exit");
  _display.display();
  delay(1500);

  while (true) {
    uint32_t pressTime = 0;
    board.readRaw(board.sensorValues);

    _display.fillRect(0, 5, 128, 30, SSD1306_BLACK);
    _display.setCursor(0, 5);
    for (int i = 0; i < 4; i++) _display.printf("%4d ", board.sensorValues[i]);
    _display.setCursor(0, 15);
    for (int i = 4; i < 8; i++) _display.printf("%4d ", board.sensorValues[i]);
    _display.setCursor(0, 26);
    for (int i = 8; i < 10; i++) _display.printf("%4d ", board.sensorValues[i]);
    _display.display();

    if (digitalRead(BUTTON_PIN) == LOW) {
      pressTime = millis();
      while (digitalRead(BUTTON_PIN) == LOW && millis() - pressTime < 2000) delay(10);
      if (millis() - pressTime >= 2000) {
        playTone(2000, 300); delay(400);
        playTone(3000, 50); delay(50);
        playTone(3000, 50); delay(50);
        delay(500);
        break;
      }
    }
    delay(100);
  }
}

void UIEsp_bordmini::viewGyroValues() {
  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setCursor(5, 10);
  _display.println("Encoder & Gyro");
  _display.setCursor(5, 25);
  _display.println("Hold 2s = Reset/Exit");
  _display.display();
  delay(1500);

  while (true) {
    uint32_t pressTime = 0;

    _display.fillRect(0, 5, 128, 30, SSD1306_BLACK);

    _display.setCursor(0, 5);
    _display.print("enL: ");
    if (encoder != nullptr) {
      _display.print(encoder->Poss_L());
    } else {
      _display.print("NONE");
    }

    _display.setCursor(70, 5);
    _display.print("enR: ");
    if (encoder != nullptr) {
      _display.print(encoder->Poss_R());
    } else {
      _display.print("NONE");
    }

    _display.setCursor(10, 15);
    _display.print("Gyro Z: ");
    if (gyro != nullptr) {
      _display.println(gyro->gyro('z'), 2);
    } else {
      _display.println("NONE");
    }

    _display.setCursor(10, 26);
    _display.print("Color: NONE");
    _display.display();

    if (digitalRead(BUTTON_PIN) == LOW) {
      pressTime = millis();
      while (digitalRead(BUTTON_PIN) == LOW && millis() - pressTime < 2000) delay(10);
      if (millis() - pressTime >= 2000) {
        if (gyro != nullptr) {
          gyro->resetAngles();
        }
        if (encoder != nullptr) {
          encoder->resetEncoders();
        }
        playTone(2000, 300); delay(400);
        playTone(3000, 50); delay(50);
        playTone(3000, 50); delay(50);
        delay(500);
        break;
      }
    }
    delay(100);
  }
}

void UIEsp_bordmini::calibrateSensors() {
  playTone(2500, 150); delay(200);
  playTone(3000, 150); delay(200);

  _display.clearDisplay();
  _display.setTextSize(2);
  _display.setCursor(20, 8);
  _display.println("CALIBRATE");
  _display.display();

  board.resetCalibration();

  uint32_t lastBeep = 0;
  uint8_t mode = 0;
  uint32_t lastSwitch = 0;

  while (true) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      uint16_t val = board.adcRead(i);
      board.sensorValues[i] = val;
      board.updateCalibration(i, val);
    }

    if (millis() - lastBeep > 1000) {
      playTone(1500, 80);
      lastBeep = millis();
    }
    if (millis() - lastSwitch > 1500) {
      mode = (mode + 1) % 3;
      lastSwitch = millis();
    }

    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setCursor(0, 0);
    if (mode == 0) _display.println("Min:");
    else if (mode == 1) _display.println("Max:");
    else _display.println("Thresh:");

    _display.setCursor(0, 10);
    for (int i = 0; i < 5; i++) {
      if (mode == 0) _display.printf("%4d ", board.adc_min(i));
      else if (mode == 1) _display.printf("%4d ", board.adc_max(i));
      else _display.printf("%4d ", (board.adc_min(i) + board.adc_max(i)) / 2);
    }
    _display.setCursor(0, 20);
    for (int i = 5; i < 10; i++) {
      if (mode == 0) _display.printf("%4d ", board.adc_min(i));
      else if (mode == 1) _display.printf("%4d ", board.adc_max(i));
      else _display.printf("%4d ", (board.adc_min(i) + board.adc_max(i)) / 2);
    }
    _display.display();

    if (digitalRead(BUTTON_PIN) == LOW) {
      delay(300);
      if (digitalRead(BUTTON_PIN) == LOW) {
        board.saveCalibrationToEEPROM();
        _display.clearDisplay();
        _display.setTextSize(2);
        _display.setCursor(30, 8);
        _display.println("SAVED!");
        _display.display();
        for (int i = 0; i < 6; i++) { playTone(3000, 100); delay(150); }
        delay(1000);
        return;
      }
    }
  }
}