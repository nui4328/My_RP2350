#pragma once
#include <Arduino.h>

// Color IDs used by the scanner and the ESP32-S3 display.
enum BandColor : uint8_t {
  BAND_RED = 0, BAND_GREEN = 1, BAND_YELLOW = 2,
  BAND_BLUE = 3, BAND_WHITE = 4, BAND_BLACK = 5
};

// Call after setup_color(), which starts Wire on Pico 2 GP4/GP5.
bool sendColorBands(BandColor first, BandColor second, BandColor third,
                    BandColor fourth, BandColor fifth, BandColor sixth);
// Send one fresh TCS color name: RED/GREEN/YELLOW/BLUE/WHITE/BLACK/UNKNOWN.
bool sendColorBands(const String& color);
bool turnOffTft();
bool turnOnTft();
bool publishScannedColors();
void resendColorBandsIfEnabled();
