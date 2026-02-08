#ifndef EncoderLibraryss_H
#define EncoderLibraryss_H

#include "Arduino.h"

class EncoderLibraryss {
public:
    EncoderLibraryss(int pinA1, int pinB1, int pinA2, int pinB2);

    void setupEncoder();

    int Poss_L();
    int Poss_R();

    void resetEncoders();

    // static instance ยังคง public เพื่อความสะดวก (หรือย้ายกลับ private ก็ได้)
    static EncoderLibraryss* _instance;

private:
    int _encoderPinA1, _encoderPinB1;
    int _encoderPinA2, _encoderPinB2;

    volatile int encoderPoss1;
    volatile int encoderPoss2;

    // ISR เป็น static
    static void encoderInterrupt3();
    static void encoderInterrupt4();

    // ให้ ISR เข้าถึง private members ได้โดยตรง
    friend void encoderInterrupt3();
    friend void encoderInterrupt4();
};

#endif