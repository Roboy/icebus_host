#include "icebus_host/IcebusHost.hpp"

int main(int argc, char *argv[]) {
  IcebusHost icebus("/dev/ttyUSB0");
  ros::Time::init();
  ros::Rate rate(100);
  int16_t pos=0;
  bool dir = true;
  int n=0;
  while(true){
    n++;
    if(n%100==0){
      icebus.SendHandStatusRequest(128);
      icebus.Listen(128);
    }
    if(dir)
      pos+=5;
    else
      pos-=5;
    if(pos>973){
      pos = 973;
      dir=!dir;
    }else if(pos<0){
      pos = 0;
      dir=!dir;
    }

    uint32_t neopxl_color=100;
    vector<uint16_t> p = {pos,pos,pos,pos};
    icebus.SendHandCommand(128,p);
    // icebus.SendHandControlMode(128);
    rate.sleep();
    ROS_INFO_THROTTLE(1,"pos %d", pos);
  }
  return 0;
}
