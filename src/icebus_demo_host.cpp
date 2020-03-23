#include "icebus_host/IcebusHost.hpp"

int main(int argc, char *argv[]) {
  if(argc!=2) {
      printf("ERROR: please provide motor config file path\n"
             "USAGE: ./icebus_demo_host path/to/motor_config.yaml\n");
      return -1;
  }
  string motor_config_file_path(argv[1]);

  IcebusHost icebus("/dev/ttyUSB0", motor_config_file_path);
  ros::Time::init();
  ros::Rate rate(100);
  int16_t pos=0;
  bool dir = true;
  int n=0;
  while(true){
    for(auto &m:icebus.motor_config->motor){
      icebus.SendStatusRequest(m.second->bus_id);
      icebus.Listen(m.second->bus_id);
    }
    rate.sleep();
  }
  return 0;
}
