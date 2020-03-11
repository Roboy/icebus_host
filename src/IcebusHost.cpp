#include "icebus_host/IcebusHost.hpp"

IcebusHost::IcebusHost(string device){
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
  tty.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds)
  tty.c_cc[VMIN] = 0; //returning as soon as this amount of data is received.
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      ROS_FATAL("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
  crcInit();
}

void IcebusHost::SendStatusRequest(int motor){
  StatusRequest req;
  req.values.header = 0xBBCEE11C;
  req.values.motor_id = motor;
  req.values.crc = gen_crc16(&req.data[4],7-4-2);
  ROS_INFO("------------");
  for(int i=0;i<sizeof(req);i++){
      printf("%x\t",req.data[i]);
  }
  printf("\n");
  write(serial_port, req.data, 7);
  // int n = read(serial_port, &read_buf, sizeof(read_buf));
  // ROS_INFO("read %d bytes",n);
  // for(int i=0;i<n;i++){
  //     printf("%x\t",read_buf[i]);
  // }
  // printf("\n");
}

void IcebusHost::SendCommand(int motor){
  Command msg;
  msg.values.header = 0xD0D0D0D0;
  msg.values.motor_id = motor;
  msg.values.setpoint = 10;
  msg.values.crc = gen_crc16(&msg.data[4],13-4-2);
  ROS_INFO("------------");
  for(int i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  write(serial_port, msg.data, 13);
}

void IcebusHost::SendControlMode(int motor){
  ControlMode msg;
  msg.values.header = 0x55AAADBA;
  msg.values.motor_id = motor;
  msg.values.control_mode = 1;
  msg.values.Kp = 0;
  msg.values.Ki = 1;
  msg.values.Kd = 2;
  msg.values.PWMLimit = 500;
  msg.values.IntegralLimit = 50;
  msg.values.deadband = 1;
  msg.values.setpoint = 1;
  msg.values.current_limit = 80;
  msg.values.crc = gen_crc16(&msg.data[4],28-4-2);
  ROS_INFO("------------");
  for(int i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  write(serial_port, msg.data, 28);
}

void IcebusHost::SendStatusResponse(int motor){
  StatusResponse msg;
  msg.values.header = 0xDA00EB1C;
  msg.values.motor_id = motor;
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
  ROS_INFO("------------");
  for(int i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  write(serial_port, msg.data, 28);
}

void IcebusHost::Listen(int motor){
  uint8_t read_buf [256];
  memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int n = read(serial_port, &read_buf, sizeof(read_buf));
  if(n>0){
    ROS_INFO("%d bytes received",n);
    switch(n){
      case 7: {
        if((read_buf[0]<<24|read_buf[1]<<16|read_buf[2]<<8|read_buf[3])==0x1CE1CEBB){
          crc crc_received = gen_crc16(&read_buf[4],n-4-2);
          if(crc_received==(read_buf[n-1]<<8|read_buf[n-2])){
            ROS_INFO("status request received for motor_id %d", read_buf[4]);
            for(int i=0;i<n;i++){
                printf("%x\t",read_buf[i]);
            }
            printf("\n");
            SendStatusResponse(motor);
          }else{
            ROS_WARN("crc dont match, crc sent %x, crc calculated %x", (read_buf[6]<<8|read_buf[5]),crc_received);
          }
        }else{
          ROS_WARN("header %x does not match",(read_buf[0]<<24|read_buf[1]<<16|read_buf[2]<<8|read_buf[3]));
        }
        break;
      }
      case 13: {
        if((read_buf[0]<<24|read_buf[1]<<16|read_buf[2]<<8|read_buf[3])==0xD0D0D0D0){
          crc crc_received = gen_crc16(&read_buf[4],n-4-2);
          if(crc_received==(read_buf[n-1]<<8|read_buf[n-2])){
            ROS_INFO("command received for motor_id %d", read_buf[4]);
            for(int i=0;i<n;i++){
                printf("%x\t",read_buf[i]);
            }
            printf("\n");
          }else{
            ROS_WARN("crc dont match, crc sent %x, crc calculated %x", (read_buf[6]<<8|read_buf[5]),crc_received);
          }
        }else{
          ROS_WARN("header %x does not match",(read_buf[0]<<24|read_buf[1]<<16|read_buf[2]<<8|read_buf[3]));
        }
        break;
      }
      case 28: {
        if((read_buf[0]<<24|read_buf[1]<<16|read_buf[2]<<8|read_buf[3])==0xBAADAA55){
          crc crc_received = gen_crc16(&read_buf[4],n-4-2);
          if(crc_received==(read_buf[n-1]<<8|read_buf[n-2])){
            ROS_INFO("control_mode received for motor_id %d", read_buf[4]);
            for(int i=0;i<n;i++){
                printf("%x\t",read_buf[i]);
            }
            printf("\n");
          }else{
            ROS_WARN("crc dont match, crc sent %x, crc calculated %x", (read_buf[6]<<8|read_buf[5]),crc_received);
          }
        }else if((read_buf[0]<<24|read_buf[1]<<16|read_buf[2]<<8|read_buf[3])==0x1CEB00DA){
          crc crc_received = gen_crc16(&read_buf[4],n-4-2);
          if(crc_received==(read_buf[n-1]<<8|read_buf[n-2])){
            ROS_INFO("status response received for motor_id %d", read_buf[4]);
            for(int i=0;i<n;i++){
                printf("%x\t",read_buf[i]);
            }
            printf("\n");
          }else{
            ROS_WARN("crc dont match, crc sent %x, crc calculated %x", (read_buf[6]<<8|read_buf[5]),crc_received);
          }
        }else{
          ROS_WARN("header %x does not match",(read_buf[0]<<24|read_buf[1]<<16|read_buf[2]<<8|read_buf[3]));
        }
        break;
      }
    }
  }
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
