#ifndef UIEsp_bordmini_h
#define UIEsp_bordmini_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MMEsp_bordmini.h"
#include "my_GYRO1601.h"
#include <EncoderLibrary.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 36
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define BUZZER_PIN 36
#define BUTTON_PIN 13

class UIEsp_bordmini {
public:
  UIEsp_bordmini(MMEsp_bordmini& boardRef, 
                 my_GYRO1601* gyroPtr = nullptr, 
                 EncoderLibrary* encoderPtr = nullptr);

  void begin();
  void playTone(int freq, int dur);
  void playStartupSound();
  void showWelcomeScreen();
  void showMainMenu();
  void waitForButtonAction();

private:
  Adafruit_SSD1306 _display;
  MMEsp_bordmini& board;
  my_GYRO1601* gyro;
  EncoderLibrary* encoder;

  void enterSettingsMode();
  void enterSensorMode();
  void viewSensorValues();
  void viewGyroValues();
  void calibrateSensors();
};

#endif