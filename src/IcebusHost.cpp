#include "icebus_host/IcebusHost.hpp"

IcebusHost::IcebusHost(string device, string motor_config_file_path){
  serial_port = open(device.c_str(), O_RDWR);

  // Check for errors
  if (serial_port < 0) {
      ROS_FATAL("Error %i from opening %s: %s\n", errno, device.c_str(), strerror(errno));
  }

  // Create new termios struc, we call it 'tty' for convention
  memset(&tty, 0, sizeof tty);

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      ROS_FATAL("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag |= PARODD; // If set, then parity for input and output is odd; otherwise even parity is used.
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds)
  tty.c_cc[VMIN] = 0; //returning as soon as this amount of data is received.
  cfsetispeed(&tty, B460800);
  cfsetospeed(&tty, B460800);
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      ROS_FATAL("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
  crcInit();

  if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "icebus_host", ros::init_options::NoSigintHandler);
  }

  nh = ros::NodeHandlePtr(new ros::NodeHandle);
  motor_config = MotorConfigPtr(new MotorConfig);
  motor_config->readConfig(motor_config_file_path);

  motorState = nh->advertise<roboy_middleware_msgs::MotorState>("/roboy/middleware/MotorState", 1);
  motorInfo = nh->advertise<roboy_middleware_msgs::MotorInfo>("/roboy/middleware/MotorInfo", 1);
  motorConfig_srv = nh->advertiseService("/roboy/middleware/MotorConfig",
                                         &IcebusHost::MotorConfigService, this);
  controlMode_srv = nh->advertiseService("/roboy/middleware/ControlMode",
                                         &IcebusHost::ControlModeService, this);
  emergencyStop_srv = nh->advertiseService("/roboy/middleware/EmergencyStop",
                                           &IcebusHost::EmergencyStopService,
                                           this);
   motorCommand_sub = nh->subscribe("/roboy/middleware/MotorCommand", 1, &IcebusHost::MotorCommand, this);

   neopixel_sub = nh->subscribe("/roboy/middleware/Neopixel", 1, &IcebusHost::Neopixel, this);

   motorStateThread = boost::shared_ptr<std::thread>(new std::thread(&IcebusHost::MotorStatePublisher, this));
    motorStateThread->detach();

    motorInfoThread = boost::shared_ptr<std::thread>(new std::thread(&IcebusHost::MotorInfoPublisher, this));
    motorInfoThread->detach();

   spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
   spinner->start();
}

void IcebusHost::Neopixel(const roboy_middleware_msgs::Neopixel::ConstPtr &msg){
  external_led_control = true;
  for (auto motor:msg->motor) {
      neopixel_color[motor] = int32_t(msg->g<<16|msg->r<<8|msg->b);
  }
}

void IcebusHost::SendStatusRequest(int id){
  StatusRequest req;
  req.values.id = id;
  req.values.crc = gen_crc16(&req.data[4],7-4-2);
  // ROS_INFO("------------");
  // for(int i=0;i<sizeof(req);i++){
  //     printf("%x\t",req.data[i]);
  // }
  // printf("\n");
  write(serial_port, req.data, 7);
  // int n = read(serial_port, &read_buf, sizeof(read_buf));
  // ROS_INFO("read %d bytes",n);
  // for(int i=0;i<n;i++){
  //     printf("%x\t",read_buf[i]);
  // }
  // printf("\n");
}

void IcebusHost::SendCommand(int id){
  Command msg;
  msg.values.id = id;
  int motor_id_global = GetGlobalID(id);
  msg.values.setpoint = swap_byte_order(setpoint[motor_id_global]);
  msg.values.neopxl_color = swap_byte_order(neopixel_color[motor_id_global]);
  msg.values.crc = gen_crc16(&msg.data[4],13-4-2);
  // ROS_INFO("------------");
  // for(int i=0;i<sizeof(msg);i++){
  //     printf("%x\t",msg.data[i]);
  // }
  // printf("\n");
  write(serial_port, msg.data, 13);
}

void IcebusHost::SendControlMode(int id){
  ControlMode msg;
  msg.values.id = id;
  int motor_id_global = GetGlobalID(id);
  msg.values.control_mode = control_mode[motor_id_global];
  msg.values.Kp = swap_byte_order16(Kp[motor_id_global]);
  msg.values.Ki = swap_byte_order16(Ki[motor_id_global]);
  msg.values.Kd = swap_byte_order16(Kd[motor_id_global]);
  msg.values.PWMLimit = swap_byte_order(PWMLimit[motor_id_global]);
  msg.values.IntegralLimit = swap_byte_order(IntegralLimit[motor_id_global]);
  msg.values.deadband = swap_byte_order(deadband[motor_id_global]);
  msg.values.setpoint = swap_byte_order(setpoint[motor_id_global]);
  int current_limit_converted = int(current_limit[motor_id_global]*80);
  msg.values.current_limit = swap_byte_order16(current_limit_converted);
  msg.values.crc = gen_crc16(&msg.data[4],28-4-2);
  // ROS_INFO("------------");
  // for(int i=0;i<sizeof(msg);i++){
  //     printf("%x\t",msg.data[i]);
  // }
  // printf("\n");
  write(serial_port, msg.data, 28);
}

void IcebusHost::SendHandStatusRequest(int id){
  HandStatusRequest msg;
  msg.values.id = id;
  msg.values.crc = gen_crc16(&msg.data[4],sizeof(msg)-4-2);
  // printf("status request------------>\t");
  // for(uint i=0;i<sizeof(msg);i++){
  //     printf("%x\t",msg.data[i]);
  // }
  // printf("\n");
  write(serial_port, msg.data, sizeof(msg));
}

void IcebusHost::SendHandCommand(int id, vector<uint16_t> pos){
  HandCommand msg;
  msg.values.id = id;
  msg.values.setpoint0 = pos[0];
  msg.values.setpoint1 = pos[1];
  msg.values.setpoint2 = pos[2];
  msg.values.setpoint3 = pos[3];
  msg.values.crc = gen_crc16(&msg.data[4],sizeof(msg)-4-2);
  // printf("command------------>\t");
  // for(uint i=0;i<sizeof(msg);i++){
  //     printf("%x\t",msg.data[i]);
  // }
  // printf("\n");
  write(serial_port, msg.data, sizeof(msg));
}

void IcebusHost::SendHandControlMode(int id){
  HandControlMode msg;
  msg.values.id = id;
  msg.values.control_mode = 1;
  msg.values.crc = gen_crc16(&msg.data[4],sizeof(msg)-4-2);
  // printf("control_mode------------>\t");
  // for(uint i=0;i<sizeof(msg);i++){
  //     printf("%x\t",msg.data[i]);
  // }
  // printf("\n");
  write(serial_port, msg.data, sizeof(msg));
}

void IcebusHost::SendHandStatusResponse(int id){
  HandStatusResponse msg;
  msg.values.id = id;
  msg.values.control_mode = 1;
  msg.values.position0 = 0;
  msg.values.current0 = 1;
  msg.values.current1 = 1;
  msg.values.current2 = 1;
  msg.values.current3 = 1;
  msg.values.setpoint0 = 2;
  msg.values.neopixel_color = 80;
  msg.values.crc = gen_crc16(&msg.data[4],sizeof(msg)-4-2);
  // printf("status_response------------>\t");
  // for(uint i=0;i<sizeof(msg);i++){
  //     printf("%x\t",msg.data[i]);
  // }
  // printf("\n");
  write(serial_port, msg.data, sizeof(msg));
}

void IcebusHost::SendStatusResponse(int id){
  StatusResponse msg;
  msg.values.id = id;
  msg.values.control_mode = 1;
  msg.values.encoder_position0 = 0;
  msg.values.encoder_position1 = 1;
  msg.values.setpoint = 2;
  msg.values.duty = 500;
  msg.values.displacement = 50;
  msg.values.current = 1;
  msg.values.setpoint = 1;
  msg.values.neopixel_color = 80;
  msg.values.crc = gen_crc16(&msg.data[4],28-4-2);
  // ROS_INFO("------------");
  // for(int i=0;i<sizeof(msg);i++){
  //     printf("%x\t",msg.data[i]);
  // }
  // printf("\n");
  write(serial_port, msg.data, 28);
}

void IcebusHost::SendM3Command(int id, int32_t setpoint){
  M3Command msg;
  msg.values.id = id;
  msg.values.setpoint = setpoint;
  msg.values.crc = gen_crc16(&msg.data[4],sizeof(msg)-4-2);
  // printf("command------------>\t");
  // for(uint i=0;i<sizeof(msg);i++){
  //     printf("%x\t",msg.data[i]);
  // }
  // printf("\n");
  write(serial_port, msg.data, sizeof(msg));
}

void IcebusHost::SendM3ControlMode(int id, uint8_t control_mode){
  M3ControlMode msg;
  msg.values.id = id;
  msg.values.control_mode = control_mode;
  msg.values.crc = gen_crc16(&msg.data[4],sizeof(msg)-4-2);
  // printf("control_mode------------>\t");
  // for(uint i=0;i<sizeof(msg);i++){
  //     printf("%x\t",msg.data[i]);
  // }
  // printf("\n");
  write(serial_port, msg.data, sizeof(msg));
}

void IcebusHost::Listen(int id){
  uint8_t read_buf [256];
  memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int n = read(serial_port, &read_buf, sizeof(read_buf));
  if(n>=7){
    // ROS_INFO("%d bytes received",n);
    // for(int i=0;i<n;i++){
    //     printf("%x\t",read_buf[i]);
    // }
    // printf("\n");
    uint32_t header = (read_buf[0]<<24|read_buf[1]<<16|read_buf[2]<<8|read_buf[3]);
    crc crc_received = gen_crc16(&read_buf[4],n-4-2);
    if(crc_received==(read_buf[n-1]<<8|read_buf[n-2])){
      switch(header){
        case 0xBBCEE11C: {
          ROS_INFO("status_request received for id %d", read_buf[4]);
          SendStatusResponse(id);
          break;
        }
        case 0xD0D0D0D0: {
          ROS_INFO("command received for id %d", read_buf[4]);
          break;
        }
        case 0x55AAADBA: {
          ROS_INFO("control_mode received for id %d", read_buf[4]);
          break;
        }
        case 0x1CEB00DA: {
          ROS_INFO_THROTTLE(5,"status_response received for id %d control_mode %d", read_buf[4], read_buf[5]);
          StatusResponse msg;
          memcpy(msg.data,read_buf,sizeof(msg));
          int motor_id_global = GetGlobalID(msg.values.id);
          encoder0_pos[motor_id_global] = interpret24bitAsInt32(&read_buf[6]);
          encoder1_pos[motor_id_global] = interpret24bitAsInt32(&read_buf[9]);
          duty[motor_id_global] = interpret24bitAsInt32(&read_buf[15]);
          displacement[motor_id_global] = interpret24bitAsInt32(&read_buf[18]);
          current[motor_id_global] = int16_t(read_buf[21]<<8|read_buf[22])/80.0f;
          if(control_mode[motor_id_global]!=read_buf[5]){
            ROS_WARN("updating control mode");
            SendControlMode(read_buf[4]);
          }
          if(setpoint[motor_id_global]!=interpret24bitAsInt32(&read_buf[12]) ||
              neopixel_color[motor_id_global]!=interpret24bitAsInt32(&read_buf[21]))
            SendCommand(read_buf[4]);
          break;
        }
        case 0xBEBAADAB: {
          ROS_INFO("hand_status_request received for id %d", read_buf[4]);
          break;
        }
        case 0x0DF005B1: {
          ROS_INFO("hand_command received for id %d", read_buf[4]);
          break;
        }
        case 0xB5006BB1: {
          ROS_INFO("hand_control_mode received for id %d", read_buf[4]);
          break;
        }
        case 0x35B1000B: {
          ROS_INFO("hand_status_response received for id %d", read_buf[4]);
          HandStatusResponse msg;
          memcpy(msg.data,read_buf,sizeof(msg));
          ROS_WARN("position: %d %d %d %d", msg.values.position0, msg.values.position1, msg.values.position2, msg.values.position3);
          ROS_WARN("current: %d %d %d %d", msg.values.current0, msg.values.current1, msg.values.current2, msg.values.current3);
          break;
        }
        case 0xCAFEBABE: {
          ROS_INFO("m3_command received for id %d", read_buf[4]);
          break;
        }
        case 0xCAFED00D: {
          ROS_INFO("m3_control_mode received for id %d", read_buf[4]);
          break;
        }
        case 0xDABAD000: {
          M3StatusResponse msg;
          memcpy(msg.data,read_buf,sizeof(msg));
          ROS_INFO_THROTTLE(10,"m3_status_response received for id %d\nsetpoint: %d\npos: %d\nvel: %d\ndis: %d\npwm: %d",
          read_buf[4], msg.values.setpoint, msg.values.pos, msg.values.vel, msg.values.dis, msg.values.pwm);
          for(auto &m:motor_config->motor){
            if(m.second->bus_id==read_buf[4]){
              encoder0_pos[m.first] = msg.values.pos;
              encoder1_pos[m.first] = msg.values.vel;
              displacement[m.first] = msg.values.dis;
              duty[m.first] = msg.values.pwm;
              if(msg.values.setpoint!=setpoint[m.first]){
                SendM3Command(read_buf[4],setpoint[m.first]);
              }
            }
          }
          break;
        }
        default: ROS_WARN_THROTTLE(5,"header %x does not match",header);
      }
    }else{
      ROS_WARN_THROTTLE(5,"crc doesn't match, crc received %x, crc calculated %x, header %x", (read_buf[n-1]<<8|read_buf[n-2]),crc_received,header);
    }
  }
}

void IcebusHost::MotorCommand(const roboy_middleware_msgs::MotorCommand::ConstPtr &msg) {
    uint i = 0;
    for (auto motor:msg->global_id) {
      auto m = motor_config->motor.find(motor);
      if( m != motor_config->motor.end()){
        if(control_mode[m->first]!=3){
          setpoint[m->first] = msg->setpoint[i];
        }else{
          bool direct_pwm_override;
          nh->getParam("direct_pwm_override",direct_pwm_override);
          if(fabsf(msg->setpoint[i])>128 && !direct_pwm_override) {
              ROS_WARN_THROTTLE(1,"setpoints exceeding sane direct pwm values (>128), "
                                  "what the heck are you publishing?!");
          }else {
              setpoint[m->first] = msg->setpoint[i];
          }
        }
      }
      i++;
    }
}

void IcebusHost::MotorStatePublisher() {
    ros::Rate rate(100);

    while (keep_publishing && ros::ok()) {
        roboy_middleware_msgs::MotorState msg;
        for (auto &m:motor_config->motor) {
            msg.global_id.push_back(m.first);
            msg.setpoint.push_back(setpoint[m.first]);
            msg.encoder0_pos.push_back(encoder0_pos[m.first]);
            msg.encoder1_pos.push_back(encoder1_pos[m.first]);
            msg.displacement.push_back(displacement[m.first]);
            msg.current.push_back(current[m.first]);
        }
        motorState.publish(msg);
        rate.sleep();
    }
}

void IcebusHost::MotorInfoPublisher() {
    ros::Rate rate(10);
    int32_t light_up_motor = 0;
    bool dir = true;
    while (keep_publishing && ros::ok()) {
        roboy_middleware_msgs::MotorInfo msg;
        int motor = 0;
        for (auto &m:motor_config->motor) {
            msg.global_id.push_back(m.first);
            msg.control_mode.push_back(control_mode[m.first]);
            msg.Kp.push_back(Kp[m.first]);
            msg.Ki.push_back(Ki[m.first]);
            msg.Kd.push_back(Kd[m.first]);
            msg.deadband.push_back(deadband[m.first]);
            msg.IntegralLimit.push_back(IntegralLimit[m.first]);
            msg.PWMLimit.push_back(PWMLimit[m.first]);
            msg.current_limit.push_back(current_limit[m.first]);
            msg.communication_quality.push_back(communication_quality[m.first]);
            msg.error_code.push_back("ok");
            msg.neopixelColor.push_back(0);
            msg.setpoint.push_back(setpoint[m.first]);
            msg.pwm.push_back(duty[m.first]);
            motor++;
        }
        motorInfo.publish(msg);
        rate.sleep();
    }
}

bool IcebusHost::MotorConfigService(roboy_middleware_msgs::MotorConfigService::Request &req,
                                     roboy_middleware_msgs::MotorConfigService::Response &res) {
    stringstream str;
    uint i = 0;
    for (int motor:req.config.global_id) {
        control_Parameters_t params;
        control_mode[motor] = req.config.control_mode[i];
        if (req.config.control_mode[i] == 0)
            str << "\t" << (int) motor << ": ENCODER0";
        if (req.config.control_mode[i] == 1)
            str << "\t" << (int) motor << ": ENCODER1";
        if (req.config.control_mode[i] == 2)
            str << "\t" << (int) motor << ": DISPLACEMENT";
        if (req.config.control_mode[i] == 3)
            str << "\t" << (int) motor << ": DIRECT_PWM";
        if(i<req.config.PWMLimit.size())
            PWMLimit[motor] = req.config.PWMLimit[i];
        if(i<req.config.Kp.size())
            Kp[motor] = req.config.Kp[i];
        if(i<req.config.Ki.size())
            Ki[motor] = req.config.Ki[i];
        if(i<req.config.Kd.size())
            Kd[motor] = req.config.Kd[i];
        if(i<req.config.deadband.size())
            deadband[motor] = req.config.deadband[i];
        if(i<req.config.IntegralLimit.size())
            IntegralLimit[motor] = req.config.IntegralLimit[i];
        if(i<req.config.update_frequency.size())
            ROS_WARN("not implemented");
        if(i<req.config.setpoint.size())
            setpoint[motor] = req.config.setpoint[i];
        ROS_INFO("setting motor %d to control mode %d with setpoint %d", motor, req.config.control_mode[i],
                 req.config.setpoint[i]);
        i++;
    }

    ROS_INFO("serving motor config service for %s control", str.str().c_str());
    return true;
}

bool IcebusHost::ControlModeService(roboy_middleware_msgs::ControlMode::Request &req,
                                     roboy_middleware_msgs::ControlMode::Response &res) {
    if (!emergency_stop) {
        if (req.global_id.empty()) {
            ROS_ERROR("no motor ids defined, cannot change control mode");
            return false;
        } else {
            int i=0;
            for (int motor:req.global_id) {
              if(motor_config->motor.find(motor) != motor_config->motor.end()){
                control_mode[motor] = req.control_mode;
                if(i<req.set_points.size()){
                  setpoint[motor] = req.set_points[i];
                }
                ROS_INFO("changing control mode of motor %d to %d", motor, req.control_mode);
              }
            }
            i++;
        }
        return true;
    } else {
        ROS_WARN("emergency stop active, can NOT change control mode");
        return false;
    }
}

bool IcebusHost::EmergencyStopService(std_srvs::SetBool::Request &req,
                                       std_srvs::SetBool::Response &res) {

    ROS_ERROR("emergency stop not implemented");
    return false;
}

void IcebusHost::crcInit(void){
    crc  remainder;
    /*
     * Compute the remainder of each possible dividend.
     */
    for (int dividend = 0; dividend < 256; ++dividend)
    {
        /*
         * Start with the dividend followed by zeros.
         */
        remainder = dividend << (WIDTH - 8);

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            /*
             * Try to divide the current data bit.
             */
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }

        /*
         * Store the result into the table.
         */
        crcTable[dividend] = remainder;
    }

}   /* crcInit() */

crc IcebusHost::gen_crc16(const uint8_t *message, uint16_t nBytes)
{
    uint8_t data;
    crc remainder = 0xffff;


    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (int byte = 0; byte < nBytes; ++byte)
    {
        data = message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    return (remainder<<8|remainder>>8);

}   /* crcFast() */
