#include "my_BNO085.h"
#include "sh2_err.h"
#include <ctype.h>
#include <math.h>
#include <string.h>

namespace {
// Two reports share the robot's 100 kHz I2C bus; 50 Hz each leaves room for
// the other sensors while keeping heading updates within the 100 ms age limit.
constexpr uint32_t REPORT_INTERVAL_US = 20000UL;
constexpr float RAD_TO_DEG_F = 180.0f / PI;
constexpr uint8_t I2C_CHUNK_SIZE = 32;

float wrapDelta(float delta) {
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

}

my_BNO085* my_BNO085::active_ = nullptr;

my_BNO085::my_BNO085(uint8_t address, TwoWire& wire)
    : _addr(address), _wire(&wire) {}

bool my_BNO085::enableReports() {
    const sh2_SensorId_t heading = _useMagnetometer
        ? SH2_ROTATION_VECTOR : SH2_GAME_ROTATION_VECTOR;
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = REPORT_INTERVAL_US;
    if (sh2_setSensorConfig(heading, &config) != SH2_OK) {
        _beginError = 3;
        return false;
    }
    if (sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config) != SH2_OK) {
        _beginError = 4;
        return false;
    }
    _beginError = 0;
    return true;
}

bool my_BNO085::begin(bool useMagnetometer) {
    if (_sessionOpen) {
        sh2_close();
        _sessionOpen = false;
    }
    active_ = this;
    _begun = false;
    _resetOccurred = false;
    _headingValid = false;
    _headingJumped = false;
    _gyroValid = false;
    _lastHeadingTimestamp = 0;
    _lastGyroTimestamp = 0;
    _angleX = _angleY = _angleZ = 0.0f;
    _freshHeadingInService = false;
    _beginError = 0;
    _lastEventId = 0;
    _eventCount = _headingEventCount = _gyroEventCount = 0;
    _otherEventCount = _decodeErrorCount = 0;
    _headingTimestampRejects = _gyroTimestampRejects = 0;
    _useMagnetometer = useMagnetometer;
    if (!probeAddress()) {
        _beginError = 1;
        return false;
    }
    _hal.open = halOpen;
    _hal.close = halClose;
    _hal.read = halRead;
    _hal.write = halWrite;
    _hal.getTimeUs = halTimeUs;
    if (sh2_open(&_hal, asyncCallback, this) != SH2_OK) {
        _beginError = 1;
        return false;
    }
    _sessionOpen = true;
    sh2_ProductIds_t productIds = {};
    if (sh2_getProdIds(&productIds) != SH2_OK) {
        _beginError = 1;
        sh2_close();
        _sessionOpen = false;
        return false;
    }
    // SH2 packets may contain both heading and gyro reports. Decode each
    // event in the callback so neither report overwrites the other.
    if (sh2_setSensorCallback(sensorCallback, this) != SH2_OK) {
        _beginError = 2;
        sh2_close();
        _sessionOpen = false;
        return false;
    }
    if (!enableReports()) {
        sh2_close();
        _sessionOpen = false;
        return false;
    }
    _resetOccurred = false;
    _begun = true;
    return true;
}

bool my_BNO085::probeAddress() {
    _wire->beginTransmission(_addr);
    return _wire->endTransmission() == 0;
}

bool my_BNO085::writePacket(const uint8_t* buffer, uint8_t length) {
    _wire->beginTransmission(_addr);
    if (_wire->write(buffer, length) != length) {
        _wire->endTransmission();
        return false;
    }
    return _wire->endTransmission() == 0;
}

bool my_BNO085::readPacket(uint8_t* buffer, uint8_t length) {
    if (_wire->requestFrom(_addr, length) != length) return false;
    for (uint8_t i = 0; i < length; ++i) {
        if (!_wire->available()) return false;
        buffer[i] = static_cast<uint8_t>(_wire->read());
    }
    return true;
}

int my_BNO085::halOpen(sh2_Hal_t*) {
    if (!active_) return -1;
    const uint8_t softReset[] = {5, 0, 1, 0, 1};
    bool sent = false;
    for (uint8_t attempt = 0; attempt < 5; ++attempt) {
        if (active_->writePacket(softReset, sizeof(softReset))) {
            sent = true;
            break;
        }
        delay(30);
    }
    if (!sent) return -1;
    delay(300);
    return 0;
}

void my_BNO085::halClose(sh2_Hal_t*) {}

int my_BNO085::halRead(sh2_Hal_t*, uint8_t* buffer, unsigned length,
                       uint32_t* timestampUs) {
    if (!active_ || !buffer) return 0;
    uint8_t header[4];
    if (!active_->readPacket(header, sizeof(header))) return 0;
    if (timestampUs) *timestampUs = micros();
    const uint16_t packetLength =
        (static_cast<uint16_t>(header[0]) |
         (static_cast<uint16_t>(header[1]) << 8)) & 0x7FFFu;
    if (packetLength < 4 || packetLength > length) return 0;

    uint16_t remaining = packetLength;
    uint8_t chunk[I2C_CHUNK_SIZE];
    bool first = true;
    while (remaining > 0) {
        const uint16_t wanted = first ? remaining : remaining + 4u;
        const uint8_t readLength = static_cast<uint8_t>(
            wanted < I2C_CHUNK_SIZE ? wanted : I2C_CHUNK_SIZE);
        if (!active_->readPacket(chunk, readLength)) return 0;
        const uint8_t skip = first ? 0 : 4;
        const uint8_t copied = readLength - skip;
        memcpy(buffer, chunk + skip, copied);
        buffer += copied;
        remaining -= copied;
        first = false;
    }
    return packetLength;
}

int my_BNO085::halWrite(sh2_Hal_t*, uint8_t* buffer, unsigned length) {
    if (!active_ || !buffer || length == 0) return 0;
    const uint8_t count = static_cast<uint8_t>(
        length < I2C_CHUNK_SIZE ? length : I2C_CHUNK_SIZE);
    return active_->writePacket(buffer, count) ? count : 0;
}

uint32_t my_BNO085::halTimeUs(sh2_Hal_t*) { return micros(); }

void my_BNO085::asyncCallback(void* cookie, sh2_AsyncEvent_t* event) {
    my_BNO085* self = static_cast<my_BNO085*>(cookie);
    if (self && event && event->eventId == SH2_RESET)
        self->_resetOccurred = true;
}

void my_BNO085::sensorCallback(void* cookie, sh2_SensorEvent_t* event) {
    my_BNO085* self = static_cast<my_BNO085*>(cookie);
    if (!self || !event) return;
    sh2_SensorValue_t value = {};
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        ++self->_decodeErrorCount;
        return;
    }
    self->handleSensorEvent(value);
}

void my_BNO085::handleSensorEvent(const sh2_SensorValue_t& value) {
    ++_eventCount;
    _lastEventId = value.sensorId;
    if (value.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        ++_gyroEventCount;
        _rawGyroX = value.un.gyroscope.x;
        _rawGyroY = value.un.gyroscope.y;
        _rawGyroZ = value.un.gyroscope.z;
        if (_collectingGyroOffset && isfinite(_rawGyroX) &&
            isfinite(_rawGyroY) && isfinite(_rawGyroZ)) {
            _calibrationSumX += _rawGyroX;
            _calibrationSumY += _rawGyroY;
            _calibrationSumZ += _rawGyroZ;
            _calibrationSquareX += _rawGyroX * _rawGyroX;
            _calibrationSquareY += _rawGyroY * _rawGyroY;
            _calibrationSquareZ += _rawGyroZ * _rawGyroZ;
            ++_calibrationCount;
        }
        acceptGyro(value.un.gyroscope, value.timestamp);
    } else if (!_useMagnetometer &&
               value.sensorId == SH2_GAME_ROTATION_VECTOR) {
        ++_headingEventCount;
        const sh2_RotationVector_t& q = value.un.gameRotationVector;
        _freshHeadingInService |= acceptHeading(q.i, q.j, q.k, q.real,
                                                value.timestamp);
    } else if (_useMagnetometer && value.sensorId == SH2_ROTATION_VECTOR) {
        ++_headingEventCount;
        const sh2_RotationVectorWAcc_t& q = value.un.rotationVector;
        _freshHeadingInService |= acceptHeading(q.i, q.j, q.k, q.real,
                                                value.timestamp);
    } else {
        ++_otherEventCount;
    }
}

void my_BNO085::acceptGyro(const sh2_Gyroscope_t& sample,
                            uint64_t timestamp) {
    if (_gyroValid && timestamp <= _lastGyroTimestamp) {
        ++_gyroTimestampRejects;
        return;
    }
    const float x = sample.x - _gyroOffsetX;
    const float y = sample.y - _gyroOffsetY;
    const float z = sample.z - _gyroOffsetZ;
    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) return;
    const float nextX = x * RAD_TO_DEG_F;
    const float nextY = y * RAD_TO_DEG_F;
    const float nextZ = z * RAD_TO_DEG_F;
    if (_gyroValid && timestamp > _lastGyroTimestamp) {
        const float dt = static_cast<float>(timestamp - _lastGyroTimestamp) * 1.0e-6f;
        // X/Y retain the old integration API, now using sensor timestamps.
        // Z comes from the fused quaternion instead.
        if (dt <= 0.1f) {
            _angleX += 0.5f * (_rateX + nextX) * dt;
            _angleY += 0.5f * (_rateY + nextY) * dt;
        }
    }
    _rateX = nextX;
    _rateY = nextY;
    _rateZ = nextZ;
    _lastGyroTimestamp = timestamp;
    _gyroValid = true;
}

bool my_BNO085::acceptHeading(float x, float y, float z, float w,
                               uint64_t timestamp) {
    const float norm2 = x*x + y*y + z*z + w*w;
    if (!isfinite(norm2) || norm2 < 0.25f) return false;
    const float inv = 1.0f / sqrtf(norm2);
    x *= inv; y *= inv; z *= inv; w *= inv;
    const float heading = atan2f(2.0f * (w*z + x*y),
                                 1.0f - 2.0f * (y*y + z*z)) * RAD_TO_DEG_F;
    if (!isfinite(heading)) return false;
    if (!_headingValid) {
        _lastHeadingDeg = heading;
        _lastHeadingTimestamp = timestamp;
        _lastHeadingAtMs = millis();
        _headingValid = true;
        return true;
    }
    if (timestamp <= _lastHeadingTimestamp) {
        ++_headingTimestampRejects;
        return false;
    }
    const float dt = static_cast<float>(timestamp - _lastHeadingTimestamp) * 1.0e-6f;
    const float delta = wrapDelta(heading - _lastHeadingDeg);
    const float rateEnvelope = _gyroValid ? fabsf(_rateZ) + 180.0f : 900.0f;
    if (dt <= 0.25f &&
        fabsf(delta) > 6.0f + rateEnvelope * dt * 2.0f) {
        // A fusion reference jump must not become a robot turn.
        _headingValid = false;
        _headingJumped = true;
        return false;
    }
    _angleZ += delta;
    _lastHeadingDeg = heading;
    _lastHeadingTimestamp = timestamp;
    _lastHeadingAtMs = millis();
    return true;
}

bool my_BNO085::update() {
    if (!_begun) return false;
    _headingJumped = false;
    _freshHeadingInService = false;
    if (_resetOccurred) {
        _resetOccurred = false;
        _headingValid = false;
        _gyroValid = false;
        _lastHeadingTimestamp = 0;
        _lastGyroTimestamp = 0;
        if (!enableReports()) {
            _begun = false;
            return false;
        }
    }
    // The callback preserves every report in each packet. Drain queued packets
    // so a slow sketch does not repeatedly process old yaw.
    for (uint8_t i = 0; i < 32; ++i) {
        const uint32_t before = _eventCount;
        sh2_service();
        if (_eventCount == before) break;
    }
    return _freshHeadingInService;
}

void my_BNO085::printDiagnostics(Print& output) const {
    output.print(F("BNO085 initError=")); output.print(_beginError);
    output.print(F(" headingEvents=")); output.print(_headingEventCount);
    output.print(F(" gyroEvents=")); output.print(_gyroEventCount);
    output.print(F(" otherEvents=")); output.print(_otherEventCount);
    output.print(F(" decodeErrors=")); output.print(_decodeErrorCount);
    output.print(F(" headingTimeRejects=")); output.print(_headingTimestampRejects);
    output.print(F(" gyroTimeRejects=")); output.print(_gyroTimestampRejects);
    output.print(F(" lastId=")); output.print(_lastEventId);
    output.print(F(" headingValid=")); output.print(_headingValid ? 1 : 0);
    output.print(F(" headingJumped=")); output.print(_headingJumped ? 1 : 0);
    output.print(F(" headingAgeMs="));
    if (_lastHeadingAtMs == 0) output.println(F("NA"));
    else output.println(static_cast<uint32_t>(millis() - _lastHeadingAtMs));
}

bool my_BNO085::angleReady(uint32_t maxAgeMs) const {
    return _headingValid &&
           static_cast<uint32_t>(millis() - _lastHeadingAtMs) <= maxAgeMs;
}

float my_BNO085::rate(char axis) const {
    switch (toupper(static_cast<unsigned char>(axis))) {
        case 'X': return _rateX;
        case 'Y': return _rateY;
        case 'Z': return _rateZ;
        default: return NAN;
    }
}

bool my_BNO085::resetYaw() {
    update();
    if (!angleReady()) return false;
    resetAngles();
    return angleReady();
}

float my_BNO085::gyro(char axis) {
    update();
    switch (toupper(static_cast<unsigned char>(axis))) {
        case 'X': return _angleX;
        case 'Y': return _angleY;
        case 'Z': return _angleZ;
        default: return NAN;
    }
}

float my_BNO085::getAngle(char axis) {
    update();
    switch (toupper(static_cast<unsigned char>(axis))) {
        case 'X': return _rateX;
        case 'Y': return _rateY;
        case 'Z': return _rateZ;
        default: return NAN;
    }
}

void my_BNO085::resetAngles() {
    update();
    _angleX = _angleY = _angleZ = 0.0f;
    // Keep the latest heading as the reference for the next delta.
    _lastGyroTimestamp = 0;
    _gyroValid = false;
}

bool my_BNO085::collectGyroOffset(uint16_t samples, uint32_t timeoutMs) {
    if (!_begun) return false;
    const uint32_t started = millis();
    _calibrationCount = 0;
    _calibrationSumX = _calibrationSumY = _calibrationSumZ = 0.0f;
    _calibrationSquareX = _calibrationSquareY = _calibrationSquareZ = 0.0f;
    _collectingGyroOffset = true;
    while (_calibrationCount < samples &&
           static_cast<uint32_t>(millis() - started) < timeoutMs) {
        update();
        delay(5);
    }
    _collectingGyroOffset = false;
    const uint16_t count = _calibrationCount;
    if (count < samples / 2u) return false;
    const float meanX = _calibrationSumX / count;
    const float meanY = _calibrationSumY / count;
    const float meanZ = _calibrationSumZ / count;
    constexpr float MAX_MEAN = 1.0f / RAD_TO_DEG_F;
    constexpr float MAX_VARIANCE = 0.25f / (RAD_TO_DEG_F * RAD_TO_DEG_F);
    if (fabsf(meanX) > MAX_MEAN || fabsf(meanY) > MAX_MEAN ||
        fabsf(meanZ) > MAX_MEAN ||
        _calibrationSquareX / count - meanX * meanX > MAX_VARIANCE ||
        _calibrationSquareY / count - meanY * meanY > MAX_VARIANCE ||
        _calibrationSquareZ / count - meanZ * meanZ > MAX_VARIANCE) return false;
    _gyroOffsetX = meanX;
    _gyroOffsetY = meanY;
    _gyroOffsetZ = meanZ;
    _headingValid = false; // Calibration consumed orientation reports.
    _gyroValid = false;
    _lastGyroTimestamp = 0;
    resetAngles();
    return true;
}

void my_BNO085::calibrate() {
    constexpr uint8_t BUZZER_PIN = 11;
    Serial.println(F("Keep BNO085 stationary during gyro offset calibration"));
    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, 1200, 120);
    delay(600);
    const bool ok = collectGyroOffset(200, 3000);
    noTone(BUZZER_PIN);
    Serial.println(ok ? F("BNO085 gyro offset ready")
                      : F("BNO085 gyro offset unavailable"));
    tone(BUZZER_PIN, ok ? 1800 : 350, ok ? 120 : 400);
}

void my_BNO085::recalibrateGyro() {
    collectGyroOffset(100, 1500);
}
