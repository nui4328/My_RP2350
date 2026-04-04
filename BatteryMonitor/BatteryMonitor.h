// BatteryMonitor.h
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
#include <Wire.h>

class BatteryMonitor {
public:
    BatteryMonitor();

    void begin();
    void update();                    // เรียกภายใน updateBattery()
    void update_led();
    float getVoltage() const;         // ค่าแรงดันที่กรองแล้ว (แนะนำใช้)
    float getRawVoltage() const;

    bool isLowBattery() const;
    bool isNoBattery() const;
    bool isNormal() const;

    void setDebug(bool enable);

private:
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

    float _voltage;
    float _rawVoltage;
    unsigned long _lastUpdate;
    unsigned long _lastBuzz;
    bool _debug;

    int16_t readADCConversion();
    void updateLEDs();
    void buzzerAlertNonBlocking();
    void writePCF(uint8_t value);
};

#endif // BATTERY_MONITOR_H