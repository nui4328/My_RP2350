#include "EncoderLibraryss.h"

// Static instance
EncoderLibraryss* EncoderLibraryss::_instance = nullptr;

EncoderLibraryss::EncoderLibraryss(int pinA1, int pinB1, int pinA2, int pinB2)
    : _encoderPinA1(pinA1), _encoderPinB1(pinB1),
      _encoderPinA2(pinA2), _encoderPinB2(pinB2),
      encoderPoss1(0), encoderPoss2(0)
{
    _instance = this;
}

void EncoderLibraryss::setupEncoder() {
    pinMode(_encoderPinA1, INPUT_PULLUP);
    pinMode(_encoderPinB1, INPUT_PULLUP);
    pinMode(_encoderPinA2, INPUT_PULLUP);
    pinMode(_encoderPinB2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(_encoderPinA1), encoderInterrupt3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_encoderPinA2), encoderInterrupt4, CHANGE);
}

int EncoderLibraryss::Poss_L() { return encoderPoss1; }
int EncoderLibraryss::Poss_R() { return encoderPoss2; }

void EncoderLibraryss::resetEncoders() {
    encoderPoss1 = 0;
    encoderPoss2 = 0;
}

// ISR ต้องใช้ scope EncoderLibraryss:: เต็ม ๆ
void EncoderLibraryss::encoderInterrupt3() {
    if (_instance == nullptr) return;

    int valA = digitalRead(_instance->_encoderPinA1);
    if (valA == HIGH) {
        if (digitalRead(_instance->_encoderPinB1) == LOW) {
            _instance->encoderPoss1--;
        } else {
            _instance->encoderPoss1++;
        }
    }
}

void EncoderLibraryss::encoderInterrupt4() {
    if (_instance == nullptr) return;

    int valA = digitalRead(_instance->_encoderPinA2);
    if (valA == HIGH) {
        if (digitalRead(_instance->_encoderPinB2) == LOW) {
            _instance->encoderPoss2--;
        } else {
            _instance->encoderPoss2++;
        }
    }
}