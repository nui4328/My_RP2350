#include "UIPiper.h"

#define BUTTON_PIN 2  // ตรงกับ PiperPico2::BUTTON_PIN

UIPiper::UIPiper(PiperPico2& piper)
  : _piper(piper),
    _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET)
{
}

void UIPiper::begin() {
    if (!_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 failed"));
        for(;;);
    }
    _display.clearDisplay();
    _display.setTextColor(SSD1306_WHITE);
    _display.setTextWrap(false);

    showSplash();
}

void UIPiper::showSplash() {
    _display.clearDisplay();

    _display.setTextSize(2);
    _display.setCursor(10, 4);
    _display.println(F("PIPER"));

    _display.setTextSize(1);
    _display.setCursor(18, 20);
    _display.println(F("ROBOTECH"));

    _display.drawRect(4, 2, 120, 28, SSD1306_WHITE);
    _display.drawFastHLine(0, 0, 128, SSD1306_WHITE);
    _display.drawFastHLine(0, 31, 128, SSD1306_WHITE);

    _display.display();
    delay(200);
}

void UIPiper::showMainMenu() {
    const char* items[] = {"Run", "Sensors", "Voltage", "Encoders", "Calibrate", "Settings"};
    uint8_t count = 6;
    uint8_t selected = 0;

    while (true) {
        _display.clearDisplay();
        _display.setTextSize(1);
        _display.setCursor(4, 2);
        _display.println(F("MAIN MENU"));

        _display.drawFastHLine(0, 9, 128, SSD1306_WHITE);

        for (uint8_t i = 0; i < count; i++) {
            uint8_t y = 12 + i * 10;
            if (y + 10 > SCREEN_HEIGHT) break;

            if (i == selected) {
                _display.fillRect(0, y-1, 128, 10, SSD1306_WHITE);
                _display.setTextColor(SSD1306_BLACK);
            } else {
                _display.setTextColor(SSD1306_WHITE);
            }
            _display.setCursor(6, y);
            _display.print(items[i]);
        }
        _display.display();

        waitForButton();

        selected = (selected + 1) % count;
    }
}

void UIPiper::waitForButton() {
    static uint32_t last = 0;
    while (true) {
        if (digitalRead(BUTTON_PIN) == LOW && millis() - last > 250) {
            last = millis();
            return;
        }
        delay(5);
    }
}

void UIPiper::showVoltage() {
    while (true) {
        float v = _piper.getVoltage();

        _display.clearDisplay();
        _display.setTextSize(1);
        _display.setCursor(0, 0);
        _display.println(F("Battery Voltage"));

        _display.setTextSize(2);
        _display.setCursor(8, 10);
        _display.print(v, 2);
        _display.print(" V");

        if (v < 10.0f) {
            _display.setTextSize(1);
            _display.setCursor(70, 24);
            _display.print(F("LOW!"));
        }

        _display.display();
        delay(200);

        if (digitalRead(BUTTON_PIN) == LOW) break;
    }
}

void UIPiper::showSensors() {
    uint16_t values[NUM_SENSORS];
    _piper.readLines(values);

    while (true) {
        _display.clearDisplay();
        _display.setTextSize(1);
        _display.setCursor(0, 0);
        _display.println(F("Line Sensors"));

        for (uint8_t i = 0; i < 5; i++) {
            _display.setCursor(i*25, 10);
            _display.print(values[i]);
        }
        for (uint8_t i = 5; i < NUM_SENSORS; i++) {
            _display.setCursor((i-5)*25, 20);
            _display.print(values[i]);
        }

        _display.display();
        delay(100);

        if (digitalRead(BUTTON_PIN) == LOW) break;
    }
}

void UIPiper::showEncoders() {
    while (true) {
        _display.clearDisplay();
        _display.setTextSize(1);
        _display.setCursor(0, 0);
        _display.println(F("Encoders"));

        _display.setCursor(0, 10);
        _display.printf("Enc1 L:%ld R:%ld", _piper.enc1Left(), _piper.enc1Right());

        _display.setCursor(0, 20);
        _display.printf("Enc2 L:%ld R:%ld", _piper.enc2Left(), _piper.enc2Right());

        _display.setCursor(0, 28);
        _display.printf("Knob:%d", (int)_piper.knopRead());

        _display.display();
        delay(150);

        if (digitalRead(BUTTON_PIN) == LOW) break;
    }
}

void UIPiper::calibrateSensorsUI() {
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setCursor(0, 10);
    _display.println(F("Calibrating..."));
    _display.println(F("Move over line"));
    _display.display();

    _piper.calibrate();

    _display.clearDisplay();
    _display.setCursor(0, 10);
    _display.println(F("Calibration Done!"));
    _display.display();
    delay(2000);
}