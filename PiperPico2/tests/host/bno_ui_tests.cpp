#define BOARD_ENABLE_GYRO 1
#define BOARD_GYRO_TYPE BOARD_GYRO_BNO055
#define BOARD_ENABLE_COLOR 0
#include "../../examples/PiperPico2_Basic/PiperPico2_Basic.ino"
#include "../../examples/PiperPico2_Basic/set_bord.ino"
#include <iostream>
static bool shown(const std::string& value){
 for(const auto& text:Adafruit_SSD1306::texts) if(text.value==value)return true;
 return false;
}
int main(){
 assert(set_bord());assert(uiGyroReady);
 assert(BNO055::address==BOARD_BNO055_ADDRESS && BNO055::loads==1);
 assert(Wire.clock==BOARD_I2C_CLOCK_HZ);
 uiSplashActive=false;uiSetPage(UI_GYRO);uiDraw();
 assert(BNO055::updates==1 && shown("BNO055 SGAM:"));
 assert(shown("Y:   123.4 deg") && shown("Z:   -12.5 deg/s"));
 BNO055::statusOK=false;uiDraw();assert(shown("????"));
 BNO055::angle=10000000.0f;uiDraw();assert(shown("Y: out of range"));
 BNO055::readOK=false;uiDraw();assert(shown("BNO055 READ ERROR"));
 assert(shown("Check I2C wiring") && !shown("Y:   123.4 deg"));
 BNO055::error=BNO055UpdateError::ImplausibleHeading;uiDraw();assert(shown("HEADING REJECTED"));
 BNO055::readOK=true;BNO055::angle=0;BNO055::statusOK=true;
 uiDraw();assert(shown("Y:     0.0 deg") && !shown("BNO055 READ ERROR"));
 uiGyroReady=false;uiDraw();assert(shown("GYRO not found"));
 uiGyroReady=true;uiSetPage(UI_RUN);const int before=BNO055::updates;
 assert(ui_update());assert(BNO055::updates==before); // RUN never polls the sensor through UI.
 BNO055::present=false;ui_begin();assert(!uiGyroReady);
 std::cout<<"BNO055 UI readings, units, errors, calibration and RUN handoff checks passed\n";
}
