#define BOARD_ENABLE_GYRO 0
#define BOARD_ENABLE_COLOR 0
#include "../../examples/PiperPico2_Basic/PiperPico2_Basic.ino"
#include "../../examples/PiperPico2_Basic/set_bord.ino"
#include <iostream>
static bool shown(const std::string& text){
 for(const auto& t:Adafruit_SSD1306::texts) if(t.value==text)return true;
 return false;
}
static void release(){gpio[2]=HIGH;ui_update();clockMs+=31;ui_update();}
static void click(){gpio[2]=LOW;ui_update();clockMs+=31;ui_update();clockMs+=100;release();}
static void hold(){gpio[2]=LOW;ui_update();clockMs+=31;ui_update();clockMs+=3000;ui_update();}
int main(){
 adcValue=0;boardReady=set_bord();assert(boardReady);assert(uiPage==UI_MAIN);
 assert(uiSplashActive && shown("PIPER"));
 assert(uiBootMelodyActive);
 for(unsigned i=0;i<sizeof(BOARD_BOOT_MELODY)/sizeof(BOARD_BOOT_MELODY[0]);++i){
  assert(lastToneFrequency==BOARD_BOOT_MELODY[i][0]);
  assert(lastToneDuration==BOARD_BOOT_MELODY[i][1]);
  const uint32_t before=clockMs;
  clockMs+=BOARD_BOOT_MELODY[i][1]+BOARD_BOOT_NOTE_GAP_MS;
  ui_update(); assert(clockMs==before+BOARD_BOOT_MELODY[i][1]+BOARD_BOOT_NOTE_GAP_MS);
 }
 assert(!uiSplashActive && !uiBootMelodyActive);
 // Short bounce cannot enter RUN.
 gpio[2]=LOW;ui_update();clockMs+=10;release();assert(uiPage==UI_MAIN);
 const uint32_t beforeRun=clockMs;
 click();assert(uiPage==UI_RUN);
 assert(lastToneFrequency==BOARD_RUN_TONE_HZ && lastToneDuration==500 && clockMs-beforeRun>=500);
 assert(shown("RUNNING") && !shown("Hold3s stop/menu"));
 robot.motor('A',30);assert(duty[8]>0);
 hold();assert(uiPage==UI_RUN && duty[8]>0);
 release();assert(uiPage==UI_RUN && duty[8]>0); // RUN ignores all later UI button input.
 uiBack();assert(uiPage==UI_RUN && duty[8]>0);
 robot.stopMotors(); // Test cleanup; real application owns control after RUN.
 uiSetPage(UI_ADC_MENU);uiSelected=3;uiDraw();
 assert(shown("adcMin") && shown("adcMD") && !shown("adcRead"));
 // All ADC values and the back hint fit in 128x32; mock asserts drawing bounds.
 uiAdcMode=0;uiSetPage(UI_ADC_VIEW);uiDraw();assert(shown("Hold3s back"));
 hold();assert(uiPage==UI_ADC_MENU);release();
 uiSetPage(UI_READ);adcValue=1400;ui_update();assert(uiSelected==1);
 click();assert(uiPage==UI_COLOR);uiDraw();assert(shown("COLOR disabled"));
 hold();assert(uiPage==UI_READ);release();
 uiSetPage(UI_GYRO);uiDraw();assert(shown("GYRO disabled"));
 uiSetPage(UI_READ);adcValue=2800;ui_update();
 assert(uiSelected==3 && shown("ENCODER") && !shown("ADC"));
 click();assert(uiPage==UI_ENCODER);
 robot.resetEncoders();
 for(uint8_t i=0;i<4;++i){
  gpio[BOARD_ENCODER_PINS[i][1]]=(i%2==0)?HIGH:LOW;
  for(uint8_t j=0;j<=i;++j)interrupts[BOARD_ENCODER_PINS[i][0]]();
 }
 uiDraw();assert(shown("1") && shown("-2") && shown("3") && shown("-4"));
 const char* labels[]={"1L:","1R:","2L:","2R:"};
 for(uint8_t i=0;i<4;++i){
  bool positionOK=false;
  for(const auto& t:Adafruit_SSD1306::texts){
   if(t.value==labels[i])positionOK=t.x==(i%2)*64 && t.y==8+(i/2)*8;
  }
  assert(positionOK);
 }
 uiDrawEncoderCell(64,16,"2R:",-2147483647L-1);assert(shown("-2147M"));
 uiDrawEncoderCell(64,16,"2R:",2147483647L);assert(shown("2147M"));
 uiDrawEncoderCell(64,16,"2R:",-999999999L);assert(shown("-1000M"));
 uiDrawEncoderCell(64,16,"2R:",-1000000L);assert(shown("-1.0M"));
 hold();assert(uiPage==UI_READ);release();
 uiSetPage(UI_CAL_MENU);adcValue=0;ui_update();click();
 assert(uiPage==UI_ADC_CAL && robot.isCalibrating());
 hold();assert(uiPage==UI_CAL_MENU && !robot.isCalibrating());release();
 uiSetPage(UI_CAL_MENU);adcValue=0;ui_update();click();
 assert(uiPage==UI_ADC_CAL);
 adcValue=3000;clockMs+=10000;ui_update();
 assert(uiPage==UI_MESSAGE && robot.calibrationSaved());assert(shown("ADC saved"));
 clockMs+=2100;ui_update();assert(uiPage==UI_CAL_MENU);
 // Clamp knob endpoints to valid indices.
 uiSetPage(UI_ADC_MENU);adcValue=4095;ui_update();assert(uiSelected==3);
 uiSetPage(UI_MAIN);adcValue=0;ui_update();assert(uiSelected==0);
 clockMs=0xfffffff0u;uiMessage("Saved","Done",UI_MAIN);clockMs+=2100;ui_update();
 assert(uiPage==UI_MAIN);
 std::cout<<"OLED navigation, debounce, calibration, RUN handoff and 128x32 layout checks passed\n";
}
