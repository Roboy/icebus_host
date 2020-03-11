#include "icebus_host/IcebusHost.hpp"

int main(int argc, char *argv[]) {
  IcebusHost icebus("/dev/ttyUSB2");
  ros::Time::init();
  while(true){
    icebus.Listen(128);
  }
  return 0;
}
