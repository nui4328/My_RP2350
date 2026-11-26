#include "PiperPico2.h"
#include <hardware/sync.h>

PiperPico2* PiperPico2::_encoderOwner = nullptr;

bool PiperPico2::setEncoderPins(uint8_t encoder, uint8_t pinA, uint8_t pinB) {
    if (_boardConfigLocked || encoder >= 4) return false;
    const uint8_t pairs[4][2] = {{20, 19}, {21, 22}, {16, 17}, {10, 18}};
    const bool normal = pinA == pairs[encoder][0] && pinB == pairs[encoder][1];
    const bool swapped = pinA == pairs[encoder][1] && pinB == pairs[encoder][0];
    if (!normal && !swapped) return false;
    _encoderPins[encoder][0] = pinA;
    _encoderPins[encoder][1] = pinB;
    return true;
}

PiperPico2::~PiperPico2() {
    if (_encoderOwner != this) return;
    const uint32_t irqState = save_and_disable_interrupts();
    for (uint8_t i = 0; i < 4; ++i) {
        detachInterrupt(digitalPinToInterrupt(_encoderPins[i][0]));
    }
    _encoderOwner = nullptr;
    restore_interrupts(irqState);
}

bool PiperPico2::initEncoders() {
    if (_encoderOwner && _encoderOwner != this) return false;
    _encoderOwner = this;
    for (uint8_t i = 0; i < 4; ++i) {
        pinMode(_encoderPins[i][0], INPUT_PULLUP);
        pinMode(_encoderPins[i][1], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(_encoderPins[0][0]), encoder0ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(_encoderPins[1][0]), encoder1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(_encoderPins[2][0]), encoder2ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(_encoderPins[3][0]), encoder3ISR, RISING);
    return true;
}

void PiperPico2::encoderEdge(uint8_t index) {
    if (_encoderOwner) {
        const uint8_t pinB = _encoderOwner->_encoderPins[index][1];
        _encoderOwner->_encoderCounts[index] += digitalRead(pinB) == LOW ? -1 : 1;
    }
}
void PiperPico2::encoder0ISR() { encoderEdge(0); }
void PiperPico2::encoder1ISR() { encoderEdge(1); }
void PiperPico2::encoder2ISR() { encoderEdge(2); }
void PiperPico2::encoder3ISR() { encoderEdge(3); }

void PiperPico2::resetEncoders() {
    // Preserve the prior IRQ state, including calls made with IRQs disabled.
    const uint32_t irqState = save_and_disable_interrupts();
    for (uint8_t i = 0; i < 4; ++i) _encoderCounts[i] = 0;
    restore_interrupts(irqState);
}
