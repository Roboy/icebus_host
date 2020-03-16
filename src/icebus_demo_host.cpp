#include "icebus_host/IcebusHost.hpp"

int main(int argc, char *argv[]) {
  IcebusHost icebus("/dev/ttyUSB0");
  ros::Time::init();
  ros::Rate rate(100);
  int16_t pos=0;
  bool dir = true;
  while(true){
    // icebus.SendHandStatusRequest(128);
    // icebus.Listen(128);
    // rate.sleep();
    if(dir)
      pos+=1;
    else
      pos-=1;
    if(pos>973){
      pos = 973;
      dir=!dir;
    }else if(pos<0){
      pos = 0;
      dir=!dir;
    }

    uint32_t neopxl_color=100;
    icebus.SendHandCommand(128,pos);
    // icebus.SendHandControlMode(128);
    rate.sleep();
    ROS_INFO_THROTTLE(1,"pos %d", pos);
  }
  return 0;
}
