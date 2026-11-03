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
   // Starting points from the TCS scan at 50 ms / 4x: black C=864..1292,
   // green starts at C=1508. Verify these against stationary can readings.
   const uint16_t blackMaxClear = 1400;
   const uint16_t whiteMinClear = 4000;
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
     const String sensorColor = tcs0.getColor();
     String measured = sensorColor;
     // Keep calibrated chromatic labels first. getColor() can return WHITE
     // early, so recover strongly colored readings from raw channels.
     if (sensorColor != "RED" && sensorColor != "GREEN" &&
         sensorColor != "BLUE" && sensorColor != "YELLOW") {
       const uint32_t red = tcs0.r;
       const uint32_t green = tcs0.g;
       const uint32_t blue = tcs0.b;
       if (tcs0.c > 200 && red * 100 >= green * 118 &&
           red * 100 >= blue * 150) {
         measured = "RED";
       } else if (tcs0.c > 200 && green * 100 >= red * 135 &&
                  green * 100 >= blue * 112) {
         measured = "GREEN";
       } else if (tcs0.c > 200 && blue * 100 >= red * 108 &&
                  blue * 100 >= green * 90) {
         measured = "BLUE";
       } else if (tcs0.c <= blackMaxClear) {
         measured = "BLACK";
       } else if (sensorColor == "WHITE" &&
                  tcs0.c >= whiteMinClear) {
         measured = "WHITE";
       } else {
         measured = "UNKNOWN";
       }
     }
     Serial.print(F("CAN: sample "));
     Serial.print(sample + 1);
     Serial.print(F(" = "));
     Serial.print(measured);
     if (measured != sensorColor) {
       Serial.print(F(" (sensor="));
       Serial.print(sensorColor);
       Serial.print(')');
     }
     Serial.print(F(" C="));
     Serial.print(tcs0.c);
     Serial.print(F(" R="));
     Serial.print(tcs0.r);
     Serial.print(F(" G="));
     Serial.print(tcs0.g);
     Serial.print(F(" B="));
     Serial.println(tcs0.b);
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
    slide_up(800);
    delay(200);
    servo(28, 140);
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
    servo(28, 150);
    for(i = 140; i > 120; i--)
      {
        servo(18, i);
        delay(15);
        servo(22, 0);
        
      }
    servo(22, 90);
    delay(500);
    servo(18, 50);
    delay(500);
  }
void griper_set()
  {
    servo(28, 10);
    delay(200);
    slide_set();
    // slide_up(100);
    servo(18, 55);
    servo(28, 90);
  }
void setGetFloor()
  {
    slide_up(500);
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
