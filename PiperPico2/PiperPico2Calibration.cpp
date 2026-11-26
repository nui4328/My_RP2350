#include "PiperPico2.h"

bool PiperPico2::startCalibration(uint32_t durationMs) {
    if (_calibrating || durationMs == 0 || durationMs > 0x7FFFFFFFUL) return false;
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        _pendingMin[i] = 4095;
        _pendingMax[i] = 0;
    }
    _calibrationStart = millis();
    _lastCalibrationSample = _calibrationStart - 50;
    _calibrationDuration = durationMs;
    _calibrationSaved = false;
    _calibrating = true;
    return true;
}

bool PiperPico2::updateCalibration() {
    if (!_calibrating) return false;
    const uint32_t now = millis();
    if (now - _lastCalibrationSample >= 50) {
        _lastCalibrationSample = now;
        for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
            const uint16_t value = adcRead(i);
            if (value < _pendingMin[i]) _pendingMin[i] = value;
            if (value > _pendingMax[i]) _pendingMax[i] = value;
        }
    }
    if (now - _calibrationStart < _calibrationDuration) return true;
    _calibrating = false;
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        if (_pendingMax[i] <= _pendingMin[i]) return false;
    }
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        _sensorMin[i] = _pendingMin[i];
        _sensorMax[i] = _pendingMax[i];
    }
    _calibrationReady = true;
    _calibrationSaved = saveCalibration();
    return false;
}

void PiperPico2::cancelCalibration() {
    _calibrating = false;
    _calibrationSaved = false;
}

void PiperPico2::calibrate() {
    if (!startCalibration()) return;
    uint32_t lastTone = millis();
    while (_calibrating) {
        updateCalibration();
        if (millis() - lastTone >= 1000) {
            playToneAsync(1200, 60);
            lastTone = millis();
        }
        delay(1);
    }
    playToneAsync(_calibrationSaved ? 3000 : 600, 200);
}

bool PiperPico2::loadCalibration() {
    // Retain the existing 40-byte layout for installed robots.
    uint16_t minimum[NUM_SENSORS], maximum[NUM_SENSORS];
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        uint8_t buffer[4] = {};
        if (!eepromRead(EEPROM_BASE_ADDR + i * 4, buffer, 4)) return false;
        minimum[i] = (uint16_t(buffer[1]) << 8) | buffer[0];
        maximum[i] = (uint16_t(buffer[3]) << 8) | buffer[2];
        if (maximum[i] > 4095 || minimum[i] >= maximum[i]) return false;
    }
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        _sensorMin[i] = minimum[i];
        _sensorMax[i] = maximum[i];
    }
    _calibrationReady = true;
    return true;
}

bool PiperPico2::saveCalibration() {
    _calibrationSaved = false;
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        if (_sensorMax[i] > 4095 || _sensorMin[i] >= _sensorMax[i]) return false;
    }
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        const uint8_t buffer[4] = {
            uint8_t(_sensorMin[i]), uint8_t(_sensorMin[i] >> 8),
            uint8_t(_sensorMax[i]), uint8_t(_sensorMax[i] >> 8)
        };
        const uint16_t addr = EEPROM_BASE_ADDR + i * 4;
        if (!eepromWrite(addr, buffer, 4)) return false;
        uint8_t verify[4] = {};
        if (!eepromRead(addr, verify, 4)) return false;
        for (uint8_t j = 0; j < 4; ++j) if (verify[j] != buffer[j]) return false;
    }
    _calibrationSaved = true;
    return true;
}

bool PiperPico2::eepromWrite(uint16_t ee_addr, const uint8_t* data, uint8_t len) {
    Wire.setClock(_i2cClock);
    Wire.beginTransmission(EEPROM_I2C_ADDR);
    Wire.write(uint8_t(ee_addr >> 8));
    Wire.write(uint8_t(ee_addr));
    for (uint8_t i = 0; i < len; ++i) Wire.write(data[i]);
    if (Wire.endTransmission() != 0) return false;
    // ACK polling avoids a fixed write-cycle delay; bounded to 20 ms.
    const uint32_t start = millis();
    do {
        Wire.beginTransmission(EEPROM_I2C_ADDR);
        if (Wire.endTransmission() == 0) return true;
        delay(1);
    } while (millis() - start < 20);
    return false;
}

bool PiperPico2::eepromRead(uint16_t ee_addr, uint8_t* data, uint8_t len) {
    Wire.setClock(_i2cClock);
    Wire.beginTransmission(EEPROM_I2C_ADDR);
    Wire.write(uint8_t(ee_addr >> 8));
    Wire.write(uint8_t(ee_addr));
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(EEPROM_I2C_ADDR, len) != len) {
        while (Wire.available()) Wire.read();
        return false;
    }
    for (uint8_t i = 0; i < len; ++i) {
        if (!Wire.available()) return false;
        data[i] = Wire.read();
    }
    return true;
}

