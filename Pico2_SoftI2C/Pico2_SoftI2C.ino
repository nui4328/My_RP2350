#include <Wire.h>
#include "SoftI2C.h"

// Original hardware I2C0 (unchanged)
constexpr uint8_t HW_SDA = 4;
constexpr uint8_t HW_SCL = 5;

// Additional software I2C bus
constexpr uint8_t SOFT_SDA = 28;
constexpr uint8_t SOFT_SCL = 1;
SoftI2C softI2c(SOFT_SDA, SOFT_SCL, 5); // nominal ~100kHz or slower

void scanHardware() {
  Serial.println("Scanning hardware I2C0 (GP4/GP5)...");
  uint8_t n = 0;
  for (uint8_t a = 0x08; a <= 0x77; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found 0x");
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
      ++n;
    }
  }
  if (!n) Serial.println("  No device found");
}

void scanSoftware() {
  Serial.println("Scanning SOFTWARE I2C (GP28/GP1)...");
  uint8_t n = 0;
  for (uint8_t a = 0x08; a <= 0x77; ++a) {
    if (softI2c.probe(a)) {
      Serial.print("  Found 0x");
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
      ++n;
    }
  }
  if (!n) Serial.println("  No device found (check 3.3V pull-ups and wiring)");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.setSDA(HW_SDA);
  Wire.setSCL(HW_SCL);
  Wire.begin();
  Wire.setClock(100000);

  softI2c.begin();
  Serial.println("Pico 2: dual I2C buses ready");
}

void loop() {
  scanHardware();
  scanSoftware();
  Serial.println("--------------------------");
  delay(3000);
}
