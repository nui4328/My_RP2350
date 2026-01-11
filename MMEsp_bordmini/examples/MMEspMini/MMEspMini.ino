#include <MMEsp_bordmini.h>
#include <UIEsp_bordmini.h>
#include <my_GYRO1601.h>
#include <EncoderLibrary.h>
#include "MoveGyroPID.h"

MMEsp_bordmini board;
my_GYRO1601 gyro;
EncoderLibrary encoder(2, 4, 5, 6);  // <<< ย้ายขึ้นมาก่อน ui
UIEsp_bordmini ui(board, &gyro, &encoder);  // ตอนนี้รู้จัก encoder แล้ว

void setup() 
  {
    set_MMEspMini();
    set_pid_moveL(2.35, 0.005, 0.03);   //------->> ตั้งค่า pid สำหรับหมุนซ้าย    ให้ปรับตัวแปรตัวสุดท้ายตัวเดียว  ถ้าหมุนเลยองศาให้เพิ่มขึ้น     **************************
    set_pid_moveR(2.45, 0.0075, 0.03);   //------->> ตั้งค่า pid สำหรับหมุนขวา    ให้ปรับตัวแปรตัวสุดท้ายตัวเดียว  ถ้าหมุนเลยองศาให้เพิ่มขึ้น     **************************
    
    move_fw(90, 1.25, 60.0);
    turnGyro(70, -90); 
    move_fw(90, 1.25, 60.0);
    turnGyro(70, -90); 
    move_fw(90, 1.25, 60.0);
    delay(1000);

    move_bw(90, 1.25, 60.0);
    turnGyro(70, 90); 
    move_bw(90, 1.25, 60.0);
    turnGyro(70, 90); 
    move_bw(90, 1.25, 60.0);
    
  }

void loop() 
  {
    // โค้ด RUN ของคุณ
      Serial.println(encoder.Poss_L());
      //Serial.println((board.adc_min(0) + board.adc_max(0)) / 2);
      //Serial.println((board.adc_min(0) + board.adc_max(0)) / 2);
      //Serial.println(gyro.gyro('z'));
      delay(10);
  }