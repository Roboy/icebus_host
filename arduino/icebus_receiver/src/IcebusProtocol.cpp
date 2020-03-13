#include "IcebusProtocol.hpp"

IcebusHost::IcebusHost(){

  crcInit();
}

void IcebusHost::SendStatusRequest(int motor){
  StatusRequest req;
  req.values.id = motor;
  req.values.crc = gen_crc16(&req.data[4],7-4-2);
  printf("status request------------>\t");
  for(int i=0;i<sizeof(req);i++){
      printf("%x\t",req.data[i]);
  }
  printf("\n");
  Serial1.write(req.data, 7);
}

void IcebusHost::SendCommand(int motor){
  Command msg;
  msg.values.id = motor;
  msg.values.setpoint = 10;
  msg.values.crc = gen_crc16(&msg.data[4],13-4-2);
  printf("command------------>\t");
  for(int i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  Serial1.write(msg.data, 13);
}

void IcebusHost::SendControlMode(int motor){
  ControlMode msg;
  msg.values.id = motor;
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
  printf("control_mode------------>\t");
  for(int i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  Serial1.write(msg.data, 28);
}

void IcebusHost::SendStatusResponse(int motor){
  StatusResponse msg;
  msg.values.id = motor;
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
  printf("status_response------------>\t");
  for(int i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  Serial1.write(msg.data, 28);
}

void IcebusHost::SendHandStatusRequest(int motor){
  HandStatusRequest msg;
  msg.values.id = motor;
  msg.values.crc = gen_crc16(&msg.data[4],sizeof(msg)-4-2);
  printf("status request------------>\t");
  for(uint i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  Serial1.write(msg.data, sizeof(msg));
}

void IcebusHost::SendHandCommand(int motor){
  HandCommand msg;
  msg.values.id = motor;
  msg.values.setpoint = 10;
  msg.values.neopxl_color = 10;
  msg.values.crc = gen_crc16(&msg.data[4],sizeof(msg)-4-2);
  printf("command------------>\t");
  for(uint i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  Serial1.write(msg.data, sizeof(msg));
}

void IcebusHost::SendHandControlMode(int motor){
  HandControlMode msg;
  msg.values.id = motor;
  msg.values.control_mode = 1;
  msg.values.crc = gen_crc16(&msg.data[4],sizeof(msg)-4-2);
  printf("control_mode------------>\t");
  for(uint i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  Serial1.write(msg.data, sizeof(msg));
}

void IcebusHost::SendHandStatusResponse(int motor){
  HandStatusResponse msg;
  msg.values.id = motor;
  msg.values.control_mode = 1;
  msg.values.position = 0;
  msg.values.current = 1;
  msg.values.setpoint = 2;
  msg.values.neopixel_color = 80;
  msg.values.crc = gen_crc16(&msg.data[4],sizeof(msg)-4-2);
  printf("status_response------------>\t");
  for(uint i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  Serial1.write(msg.data, sizeof(msg));
}

void IcebusHost::Listen(int id){
  uint8_t read_buf [256];
  memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int n=Serial1.available();
  if (n>=7) {
    for(int i=0;i<n;i++){
      read_buf[i] = Serial1.read();
    }
    printf("%d bytes received\n",n);
    for(int i=0;i<n;i++){
        printf("%x\t",read_buf[i]);
    }
    printf("\n");
  }
  if(n>0){
    uint32_t header = (read_buf[0]<<24|read_buf[1]<<16|read_buf[2]<<8|read_buf[3]);
    crc crc_received = gen_crc16(&read_buf[4],n-4-2);
    if(crc_received==(read_buf[n-1]<<8|read_buf[n-2])){
      switch(header){
        case 0x1CE1CEBB: {
          printf("status_request received for id %d", read_buf[4]);
          SendStatusResponse(id);
          break;
        }
        case 0xD0D0D0D0: {
          printf("command received for id %d", read_buf[4]);
          break;
        }
        case 0xBAADAA55: {
          printf("control_mode received for id %d", read_buf[4]);
          break;
        }
        case 0x1CEB00DA: {
          printf("status_response received for id %d", read_buf[4]);
          break;
        }
        case 0xABADBABE: {
          printf("hand_status_request received for id %d", read_buf[4]);
          break;
        }
        case 0xB105F00D: {
          printf("hand_command received for id %d", read_buf[4]);
          break;
        }
        case 0xB16B00B5: {
          printf("hand_control_mode received for id %d", read_buf[4]);
          break;
        }
        case 0x0B00B135: {
          printf("hand_status_response received for id %d", read_buf[4]);
          break;
        }
        default: printf("header %x does not match",header);
      }
    }else{
      printf("crc doesn't match, crc received %x, crc calculated %x, header %x", (read_buf[7]<<8|read_buf[6]),crc_received,header);
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
