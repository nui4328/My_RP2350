#ifndef UIPIPER_H
#define UIPIPER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "PiperPico2.h"

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

#define NUM_SENSORS 10  // ตรงกับ PiperPico2

#define BUTTON_PIN 2    // ตรงกับ PiperPico2::BUTTON_PIN

class UIPiper {
public:
    UIPiper(PiperPico2& piper);

    void begin();
    void showSplash();
    void showMainMenu();
    void showVoltage();
    void showSensors();
    void showEncoders();
    void calibrateSensorsUI();
    void waitForButton();

private:
    PiperPico2& _piper;
    Adafruit_SSD1306 _display;
};

#endif