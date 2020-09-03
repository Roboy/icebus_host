#include "icebus_host/IcebusHost.hpp"

int main(int argc, char *argv[]) {
  IcebusHost icebus("/dev/ttyUSB0");
  ros::Time::init();
  while(true){
    icebus.Listen(131);
  }
  return 0;
}
