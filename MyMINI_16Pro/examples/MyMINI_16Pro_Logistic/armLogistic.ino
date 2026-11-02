void slide_set()
  {
    do{servo(22, 0);Serial.println(digitalRead(1));}while(digitalRead(1) == 1);
    servo(22, 90);
    delay(200);
    do{servo(22, 180);Serial.println(digitalRead(1));}while(digitalRead(1) == 0);
    servo(22, 90);
  }
void slide_down(int time)
  {
    servo(22, 0);
    delay(time);
    servo(22, 90);
  }
void slide_up(int time)
  {
    servo(22, 180);
    delay(time);
    servo(22, 90);
  }
void set_can()
  {
    servo(18, 0);
  }
void getCan()
  {
    _can_color = "";  // Never reuse the previous can's color after a failed read.
    servo(18, 130);
    delay(200);
   servo(28, 60);
   delay(300);
   static const char* const names[6] = {
       "RED", "GREEN", "YELLOW", "BLUE", "WHITE", "BLACK"};
   uint8_t votes[6] = {};
   uint8_t readFailures = 0;
   // TCS integration is 50 ms. Space reads so repeated votes come from
   // separate conversions while the can is held in front of the sensor.
   for (uint8_t sample = 0; sample < 5; ++sample) {
     delay(60);
     if (!tcs0.readFast()) {
       ++readFailures;
       continue;
     }
     const String measured = tcs0.getColor();
     Serial.print(F("CAN: sample "));
     Serial.print(sample + 1);
     Serial.print(F(" = "));
     Serial.print(measured);
     Serial.print(F(" C="));
     Serial.println(tcs0.c);
     for (uint8_t color = 0; color < 6; ++color) {
       if (measured == names[color]) {
         ++votes[color];
         break;
       }
     }
   }
   uint8_t bestVotes = 0;
   int8_t bestColor = -1;
   for (uint8_t color = 0; color < 6; ++color) {
     if (votes[color] > bestVotes) {
       bestVotes = votes[color];
       bestColor = color;
     }
   }
   if (bestColor >= 0 && bestVotes >= 3) {
     _can_color = names[bestColor];
     Serial.print(F("CAN: color = "));
     Serial.println(_can_color);
     if (!sendColorBands(_can_color))
       Serial.println(F("COLOR I2C: can color send failed"));
   } else {
     Serial.print(F("CAN: color uncertain; best votes="));
     Serial.print(bestVotes);
     Serial.print(F("/5, read failures="));
     Serial.println(readFailures);
   }
    delay(200);
    servo(28, 90);
    slide_up(600);
    delay(200);
    servo(28, 120);
  }
void drop()
  {
    servo(18, 30);
    delay(500);
    servo(18, 0);
    delay(300);
  }
void slide_drop()
  {
    int i ;
    for(i = 140; i > 120; i--)
      {
        servo(18, i);
        delay(20);
        servo(22, 0);
        
      }
    servo(22, 90);
    delay(500);
    servo(18, 50);
    delay(200);
  }
void griper_set()
  {
    servo(28, 10);
    delay(200);
    slide_set();
    slide_up(100);
    servo(18, 55);
    servo(28, 90);
  }
void setGetFloor()
  {
    slide_up(600);
    servo(28, 120);
  }
void GetFloor()
  {
    servo(18, 140);
    delay(200);
    slide_up(200);
  }
void getInFloor()
  {
    slide_down(200);
    delay(200);
    servo(18, 140);
    delay(200);
    slide_up(700);
  }
