#include <myrp_pico2.h>

//----------------------------------------------------------------------------->> ตั้งค่ามือจับ
int servo_down = 35;        //-------------------->> ตั้งค่า มื่อจับลงขนานกับพื้น
int servoL_open = 120;      //-------------------->> ตั้งค่า กางฝ่ามือด้านซ้าย
int servoR_open = 120;      //-------------------->> ตั้งค่า กางฝ่ามือด้านขวา
//----------------------------------------------------------------------------->> ตั้งค่ามือจับ
//----------------------------------------------------------------------------->> ตั้งค่ามือจับ



void setup() 
  {
    set_motor();
    setup_robot() ;
    
    
    //----------------------------------------------------------------------------->> ตั้งค่ามือจับ
   // s34_trim(0);
   // s35_trim(10);
   // s36_trim(0);
  //----------------------------------------------------------------------------->> ตั้งค่ามือจับ
    arm_down_open();
   //------------------------->>  ส่วนของคำสั่งในการเตรียมแขนกล 
 
    sw();  //--->> คำสั่งรอกดปุ่ม
    //resetAngles();
   
    ////------------------------------------------------------------------------------>> รันคำสั่งต่าง ๆ ที่นี่
     
     rotate(-30, 30, -45, 10) ;


     sensor_position( 30);
     fline(50, 50, 0.25, "fr", 'c', 'r', 60, "a6", 10);
     sw();
     fline(30, 30, 0.55, "fl", 'c', 'l', 60, "a1", 10);
    
   


    

    ////------------------------------------------------------------------------------>> จบการรันคำสั่งต่าง ๆ 
  
  }

void loop() 
  {
    //  Motor(30, 30);
     //updateBattery();
      
    //bat.update();
     //Serial.print(); Serial.println( "   " ); 
    //Motor(30, 30);servo(10, 40);servo(1, 40);servo(0, 40);delay(1000);
    //Motor(-30, -30);servo(10, 140);servo(1, 140);servo(0, 140);delay(1000);
    //Motor(30, 30); delay(1000);
    //Motor(-30, -30); delay(1000);
     //Serial.print(encoder.Poss_L());Serial.print("  "); Serial.println(encoder.Poss_R());

    //Serial.print("adc :");Serial.print(ADC_i2c()); Serial.println( "   " );  
   
    Serial.print("my.gyro('z') :");Serial.print(my.gyro('z')); Serial.print( "   " ); 
    Serial.print( analogRead(26) );   Serial.print( "  " );   Serial.print( analogRead(27) ); Serial.println( "     " );
    //Serial.print(  md_sensorC(0));   Serial.print( "  " );   Serial.println(  md_sensorC(1) ); 

    // Serial.print( sensorMinC[0] );   Serial.print( "   " );   Serial.print( sensorMinC[1] ); 
    //Serial.print( "   " ); 
    // Serial.print( sensorMaxC[0] );   Serial.print( "   " );   Serial.println( sensorMaxC[1] );  
    
    delay(10);
     //Serial.print(sensorMin_A[0] );

  }
