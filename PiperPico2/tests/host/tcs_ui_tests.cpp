#define BOARD_ENABLE_COLOR 1
#define BOARD_ENABLE_GYRO 0
#include "../../examples/PiperPico2_Basic/PiperPico2_Basic.ino"
#include "../../examples/PiperPico2_Basic/set_bord.ino"
#include <iostream>
static void release(){gpio[2]=HIGH;ui_update();clockMs+=31;ui_update();}
static void click(){gpio[2]=LOW;ui_update();clockMs+=31;ui_update();clockMs+=100;release();}
static void choose(uint8_t sensor){
 uiSetPage(UI_TCS_MENU);adcValue=sensor==0?0:2800;ui_update();
 assert(uiSelected==sensor);click();
}
static bool shown(const std::string& s){
 for(const auto& t:Adafruit_SSD1306::texts)if(t.value==s)return true;
 return false;
}
int main(){
 assert(set_bord());uiSplashActive=uiBootMelodyActive=false;
 uiSetPage(UI_CAL_MENU);uiSelected=1;uiEnter();assert(uiPage==UI_TCS_MENU);
 uiDraw();assert(shown("TCS0-Cal") && shown("TCS1-Cal"));
 choose(0);assert(uiPage==UI_COLOR_CAL && uiColorSensor==0 && uiColorStep==0);
 assert(my_TCS::starts[0]==1 && my_TCS::starts[1]==0);
 gpio[2]=LOW;ui_update();clockMs+=31;ui_update();clockMs+=1000;ui_update();
 assert(my_TCS::captures[0]==0);release();assert(my_TCS::captures[0]==1 && uiColorStep==1);
 for(int i=1;i<6;++i)click();
 assert(uiPage==UI_MESSAGE && shown("TCS0 saved"));
 assert(my_TCS::saves[0]==1 && my_TCS::saves[1]==0 && my_TCS::starts[1]==0);
 clockMs+=2100;ui_update();assert(uiPage==UI_TCS_MENU); // Never auto-start the other sensor.
 choose(1);assert(uiColorSensor==1 && uiColorStep==0);
 for(int i=0;i<6;++i)click();
 assert(shown("TCS1 saved") && my_TCS::saves[1]==1);
 assert(my_TCS::saves[0]==1 && my_TCS::starts[0]==1);
 // Cancelling TCS0 reloads only TCS0, not TCS1.
 choose(0);click();
 gpio[2]=LOW;ui_update();clockMs+=31;ui_update();clockMs+=3000;ui_update();
 assert(uiPage==UI_TCS_MENU && my_TCS::loads[0]==1 && my_TCS::loads[1]==0);
 release();assert(uiPage==UI_TCS_MENU);
 choose(1);my_TCS::readOK=false;click();
 assert(uiColorStep==0 && uiColorRetry);
 my_TCS::readOK=true;uiTcs1.c=0;click();assert(uiColorStep==0);
 uiTcs1.c=100;click();assert(uiColorStep==1 && !uiColorRetry);
 my_TCS::saveOK=false;for(int i=1;i<6;++i)click();
 assert(shown("TCS save failed") && uiMessageReturn==UI_TCS_MENU);
 uiColorReady[1]=false;choose(1);assert(shown("TCS1 not found"));
 assert(uiPage==UI_MESSAGE && my_TCS::starts[0]==2);
 uiColorReady[1]=true;my_TCS::captureOK=false;choose(0);
 for(int i=0;i<6;++i)click();
 assert(shown("Capture failed"));
 assert(my_TCS::loads[0]==2 && my_TCS::loads[1]==0);
 std::cout<<"Independent TCS0/TCS1 calibration, short clicks, cancel and failure checks passed\n";
}
