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
  bool toggle = true;
  int n=0;
  ros::Time t0=ros::Time::now();
  while(true){
    for(auto &m:icebus.motor_config->motor){
      ROS_INFO_THROTTLE(100,"Sending status request to motor %d with bus_id %d.", m.first, m.second->bus_id);
      icebus.SendStatusRequest(m.second->bus_id);
      icebus.Listen(m.second->bus_id);
    }
    rate.sleep();
    if((ros::Time::now()-t0).toSec()>1 && !icebus.external_led_control){
      t0 = ros::Time::now();
      for(auto &m:icebus.motor_config->motor){
        if(toggle)
          icebus.neopixel_color[m.first] = 50;
        else
          icebus.neopixel_color[m.first] = 0;
        toggle = !toggle;
      }
    }
  }
  return 0;
}
