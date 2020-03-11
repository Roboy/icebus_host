#include "icebus_host/IcebusHost.hpp"

int main(int argc, char *argv[]) {
  IcebusHost icebus("/dev/ttyUSB0");
  ros::Time::init();
  ros::Rate rate(1);

  while(true){
    icebus.SendStatusRequest(128);
    usleep(10000);
    icebus.Listen(128);
    icebus.SendCommand(128);
    usleep(10000);
    icebus.SendControlMode(128);
    usleep(10000);
    rate.sleep();
  }
  return 0;
}
