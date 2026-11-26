#define BOARD_ENABLE_COLOR 1
#define BOARD_ENABLE_GYRO 1
#define BOARD_GYRO_TYPE BOARD_GYRO_BNO055
#include "../../examples/PiperPico2_Basic/PiperPico2_Basic.ino"
#include "../../examples/PiperPico2_Basic/set_bord.ino"
#include <iostream>
static bool shown(const std::string& s){
 for(const auto& t:Adafruit_SSD1306::texts)if(t.value==s)return true;
 return false;
}
int main(){
 Adafruit_PWMServoDriver::works=false;
 assert(!set_bord());
 assert(shown("BOARD INIT FAILED") && shown("ERROR"));
 assert(duty[8]==0 && duty[9]==0 && duty[7]==0 && duty[6]==0);
 Adafruit_PWMServoDriver::works=true;
 assert(robot.begin());ui_begin();
 const bool conflict=BOARD_BNO055_ADDRESS==TCS34725_ADDRESS;
 assert(uiAddressConflict==conflict);
 if(conflict){
  assert(!uiGyroReady && !uiColorReady[0] && uiColorReady[1]);
  assert(BNO055::beginCalls==0 && my_TCS::beginCalls[0]==0 && my_TCS::beginCalls[1]==1);
  uiDraw();assert(shown("ADDR CONFLICT 0x29"));
  uiSetPage(UI_GYRO);uiDraw();assert(shown("ADDR CONFLICT 0x29"));
  assert(BNO055::updates==0);
  uiSetPage(UI_COLOR);uiDraw();assert(shown("ADDR CONFLICT"));
  uiSetPage(UI_TCS_MENU);uiSelected=0;uiEnter();assert(uiPage==UI_MESSAGE);
  assert(shown("ADDR CONFLICT") || uiMessageTitle==std::string("ADDR CONFLICT 0x29"));
 }else{
  assert(uiGyroReady && uiColorReady[0] && uiColorReady[1]);
 }
 std::cout<<"Boot failure diagnostics and shared-address conflict guard passed\n";
}
