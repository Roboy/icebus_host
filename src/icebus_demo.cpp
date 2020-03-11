#include "icebus_host/IcebusHost.hpp"

int main(int argc, char *argv[]) {
  IcebusHost icebus;
  ros::Time::init();
  ros::Rate rate(1);
  while(true){
    icebus.GetStatus(128);
    rate.sleep();
  }
  return 0;
}
