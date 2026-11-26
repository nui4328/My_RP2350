"""Compile and test the actual getColor() body from the installed my_TCS source.
An optional source path supports reviewing a candidate before installing it.
Hardware/I2C code is excluded; only classification is exercised.
"""
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parent.parent
source = Path(sys.argv[1]) if len(sys.argv) > 1 else root.parent / 'my_TCS' / 'my_TCS.cpp'
text = source.read_text(encoding='utf-8-sig')
start = text.index('String my_TCS::getColor() {')
end = text.index('void my_TCS::printRaw()', start)
method = text[start:end]
build = root / '.build'
build.mkdir(exist_ok=True)
cpp = build / 'tcs_color_classifier_test.cpp'
exe = build / 'tcs_color_classifier_test.exe'
cpp.write_text(r"""
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
using String = std::string;
struct my_TCS {
 bool calibrated=true;
 uint16_t c=0,r=0,g=0,b=0;
 bool cal_flags[6]={true,true,true,true,true,true};
 float ref_ratio_r[6]={.65f,.20f,.15f,.45f,.33f,.33f};
 float ref_ratio_g[6]={.20f,.65f,.20f,.45f,.33f,.33f};
 float ref_ratio_b[6]={.15f,.15f,.65f,.10f,.34f,.34f};
 String getColor();
 void sample(uint16_t clear,float red,float green,float blue){
  c=clear;r=uint16_t(std::round(clear*red));g=uint16_t(std::round(clear*green));b=uint16_t(std::round(clear*blue));
 }
};
""" + method + r"""
int main(){
 my_TCS tcs0,tcs1;
 // Bright colors must not be taken by WHITE; dim chromatic matches must
 // win before BLACK, including those found only by the distance fallback.
 for(my_TCS* sensor:{&tcs0,&tcs1}){
  sensor->sample(3000,.65f,.20f,.15f);assert(sensor->getColor()=="RED");
  sensor->sample(100,.65f,.20f,.15f);assert(sensor->getColor()=="RED");
  sensor->sample(3000,.20f,.65f,.15f);assert(sensor->getColor()=="GREEN");
  sensor->sample(100,.20f,.65f,.15f);assert(sensor->getColor()=="GREEN");
  sensor->sample(3000,.15f,.20f,.65f);assert(sensor->getColor()=="BLUE");
  sensor->sample(100,.15f,.20f,.65f);assert(sensor->getColor()=="BLUE");
  sensor->sample(3000,.45f,.45f,.10f);assert(sensor->getColor()=="YELLOW");
  sensor->sample(100,.45f,.45f,.10f);assert(sensor->getColor()=="YELLOW");
  sensor->sample(2000,.33f,.33f,.34f);assert(sensor->getColor()=="WHITE");
  sensor->sample(100,.33f,.33f,.34f);assert(sensor->getColor()=="BLACK");
  sensor->sample(1000,.33f,.33f,.34f);assert(sensor->getColor()=="UNKNOWN");
  sensor->c=2000;sensor->r=sensor->g=sensor->b=0;assert(sensor->getColor()=="WHITE");
  sensor->c=0;assert(sensor->getColor()=="BLACK"); // No division by zero.
  sensor->sample(29,.33f,.33f,.34f);assert(sensor->getColor()=="BLACK");
  sensor->sample(180,.33f,.33f,.34f);assert(sensor->getColor()=="UNKNOWN");
  sensor->sample(1800,.33f,.33f,.34f);assert(sensor->getColor()=="UNKNOWN");
  sensor->sample(1801,.33f,.33f,.34f);assert(sensor->getColor()=="WHITE");
  sensor->cal_flags[4]=false;assert(sensor->getColor()=="UNKNOWN");
  sensor->cal_flags[5]=false;sensor->c=0;assert(sensor->getColor()=="UNKNOWN");
  sensor->calibrated=false;sensor->sample(3000,.65f,.20f,.15f);assert(sensor->getColor()=="UNKNOWN");
 }
 std::cout<<"Actual my_TCS getColor(): color-first, C-only white/black, boundaries and both sensors passed\n";
}
""", encoding='utf-8')
subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(cpp),'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)
