// BatteryMonitor.h
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
#include <Wire.h>

class BatteryMonitor {
public:
  BatteryMonitor();  // constructor

  // เริ่มต้นใช้งาน (เรียกใน setup())
  void begin();

  // อัพเดทสถานะ (เรียกใน loop() ทุกครั้งที่ต้องการตรวจสอบ)
  void update();

  // ดึงค่าแรงดันล่าสุด
  float getVoltage() const;
  float readBatteryVoltage();

  // ตรวจสอบสถานะต่าง ๆ
  bool isLowBattery() const;       // < V_EMPTY แต่ >= V_MIN_VALID
  bool isNoBattery() const;        // < V_MIN_VALID
  bool isNormal() const;

private:
  // ── ค่าคงที่ (สามารถปรับได้ใน .cpp) ──────────────────────────────
  static const uint8_t ADS_ADDR;
  static const uint8_t PCF_ADDR;
  static const uint8_t BUZZER_PIN;

  static const float CAL_SCALE;
  static const float CAL_OFFSET;

  static const float V_FULL;
  static const float V_EMPTY;
  static const float V_MIN_VALID;
  static const float V_STEP;

  static const uint16_t ADC_CONFIG;

  // ตัวแปรสถานะภายใน
  float _voltage;
  unsigned long _lastUpdate;
  unsigned long _lastBuzz;

  // ฟังก์ชันภายใน
  
  int16_t readADCConversion();
  void updateLEDs();
  void buzzerAlert();
  void writePCF(uint8_t value);
};

#endif // BATTERY_MONITOR_H