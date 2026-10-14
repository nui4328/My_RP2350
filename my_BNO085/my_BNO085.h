#ifndef MY_BNO085_H
#define MY_BNO085_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

class my_BNO085 {
public:
    my_BNO085(uint8_t address = 0x4B, TwoWire& wire = Wire1);
    // Select the I2C controller before begin().
    bool selectWire(TwoWire& wire) {
        if (_begun) return false;
        _wire = &wire;
        return true;
    }
    // Game Rotation Vector by default; true uses magnetic-north Rotation Vector.
    // The sketch must start the selected Wire bus before begin().
    bool begin(bool useMagnetometer = false);
    bool update();  // True when a new, valid heading was read.
    bool angleReady(uint32_t maxAgeMs = 100) const;
    bool headingJumped() const { return _headingJumped; }
    float yaw() const { return _angleZ; }
    float yawRaw() const { return _angleZ; }
    float rate(char axis) const; // Degrees/second without another I2C read.
    bool resetYaw();
    float gyro(char axis);     // Existing API: accumulated angle (degrees).
    float getAngle(char axis); // Existing API: rate (degrees/second).
    void resetAngles();
    void printDiagnostics(Print& output) const;
    void calibrate();
    void recalibrateGyro();

private:
    bool enableReports();
    bool collectGyroOffset(uint16_t samples, uint32_t timeoutMs);
    bool acceptHeading(float x, float y, float z, float w, uint64_t timestamp);
    void acceptGyro(const sh2_Gyroscope_t& sample, uint64_t timestamp);
    static void sensorCallback(void* cookie, sh2_SensorEvent_t* event);
    void handleSensorEvent(const sh2_SensorValue_t& value);
    Adafruit_BNO08x bno;
    uint8_t _addr;
    TwoWire* _wire;
    bool _begun = false;
    bool _useMagnetometer = false;
    bool _headingValid = false;
    bool _headingJumped = false;
    bool _gyroValid = false;
    bool _freshHeadingInService = false;
    uint8_t _beginError = 0; // 1=SH2 init, 2=callback, 3=heading, 4=gyro
    uint8_t _lastEventId = 0;
    uint32_t _eventCount = 0;
    uint32_t _headingEventCount = 0;
    uint32_t _gyroEventCount = 0;
    uint32_t _otherEventCount = 0;
    uint32_t _decodeErrorCount = 0;
    uint32_t _headingTimestampRejects = 0;
    uint32_t _gyroTimestampRejects = 0;
    float _rawGyroX = 0.0f;
    float _rawGyroY = 0.0f;
    float _rawGyroZ = 0.0f;
    bool _collectingGyroOffset = false;
    uint16_t _calibrationCount = 0;
    float _calibrationSumX = 0.0f, _calibrationSumY = 0.0f;
    float _calibrationSumZ = 0.0f;
    float _calibrationSquareX = 0.0f, _calibrationSquareY = 0.0f;
    float _calibrationSquareZ = 0.0f;
    float _gyroOffsetX = 0.0f; // radians/second
    float _gyroOffsetY = 0.0f;
    float _gyroOffsetZ = 0.0f;
    float _rateX = 0.0f; // degrees/second
    float _rateY = 0.0f;
    float _rateZ = 0.0f;
    float _angleX = 0.0f;
    float _angleY = 0.0f;
    float _angleZ = 0.0f; // continuous fused yaw, beyond +/-360 degrees
    float _lastHeadingDeg = 0.0f;
    uint64_t _lastHeadingTimestamp = 0;
    uint64_t _lastGyroTimestamp = 0;
    uint32_t _lastHeadingAtMs = 0;
};

#endif
