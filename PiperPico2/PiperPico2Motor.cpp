#include "PiperPico2.h"

bool PiperPico2::setMotorDriver(char id, MotorDriver driver, bool reverseDirection) {
    if (_boardConfigLocked || id < 'A' || id > 'D') return false;
    if (driver != MotorDriver::LEGACY && driver != MotorDriver::DRV8874) return false;
    _motorDrivers[id - 'A'] = driver;
    _motorReversed[id - 'A'] = reverseDirection;
    return true;
}

void PiperPico2::invalidateMotorCache() {
    for (uint8_t i = 0; i < 4; ++i) _motorDirections[i] = 2;
}

void PiperPico2::stopMotors() {
    bool ok = true;
    for (char id = 'A'; id <= 'D'; ++id) {
        motor(id, 0);
        ok = _motorOK && ok;
    }
    _motorOK = ok;
}

void PiperPico2::motor(char id, int speed) {
    if (id < 'A' || id > 'D') return;
    const uint8_t pins[4] = {PWM_PIN_A, PWM_PIN_B, PWM_PIN_C, PWM_PIN_D};
    const uint8_t index = id - 'A';
    speed = constrain(speed, -100, 100);
    if (_motorReversed[index]) speed = -speed;
    const int8_t direction = (speed > 0) - (speed < 0);
    if (!_pwmReady) {
        analogWrite(pins[index], 0);
        _motorOK = false;
        return;
    }
    if (_motorDirections[index] != direction) {
        // Remove drive before changing either direction input.
        analogWrite(pins[index], 0);
        Wire.setClock(_i2cClock);
        _motorOK = writeMotorDirection(index, id, direction);
        if (!_motorOK) {
            _motorDirections[index] = 2; // Retry on the next call.
            return;
        }
        _motorDirections[index] = direction;
    }
    _motorOK = true;
    const uint16_t duty = static_cast<uint32_t>(abs(speed)) * PWM_MAX / 100U;
    analogWrite(pins[index], duty);
}

bool PiperPico2::writeMotorDirection(uint8_t index, char id, int8_t direction) {
    if (_motorDrivers[index] == MotorDriver::DRV8874) {
        // Schematic nets ledN are PCA9685 channels N.
        // A: PH=0, nSLEEP=1; C: PH=2, nSLEEP=3.
        // B/D extend the same adjacent-pair layout to 4/5 and 6/7.
        const uint8_t phaseChannels[4] = {0, 4, 2, 6};
        const uint8_t sleepChannels[4] = {1, 5, 3, 7};
        const uint8_t phase = phaseChannels[index];
        const uint8_t sleep = sleepChannels[index];
        // EN is already LOW here, so changes cannot drive the motor.
        if (_pwm.setPWM(phase, direction > 0 ? 4096 : 0, 0) != 0) return false;
        if (_motorDirections[index] == 2) {
            // Initial setup, an earlier failure, or externally invalidated state.
            if (_pwm.setPWM(sleep, 4096, 0) != 0) return false;
            delay(1); // DRV8874 tWAKE: wait before enabling EN PWM.
        }
        // Keep nSLEEP HIGH: EN=0 brakes; repeated starts need no wake delay.
        return true;
    }
    const bool first = _pwm.setPWM(getMotorIn1(id), direction > 0 ? 4096 : 0, 0) == 0;
    const bool second = _pwm.setPWM(getMotorIn2(id), direction < 0 ? 4096 : 0, 0) == 0;
    return first && second;
}

uint8_t PiperPico2::getMotorIn1(char id) {
    switch (id) {
        case 'A': return 1;
        case 'B': return 5;
        case 'C': return 2;
        case 'D': return 6;
        default: return 255;
    }
}

uint8_t PiperPico2::getMotorIn2(char id) {
    switch (id) {
        case 'A': return 0;
        case 'B': return 4;
        case 'C': return 3;
        case 'D': return 7;
        default: return 255;
    }
}



