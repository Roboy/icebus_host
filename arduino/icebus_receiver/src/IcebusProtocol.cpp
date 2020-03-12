#include "IcebusProtocol.hpp"

IcebusHost::IcebusHost(){

  crcInit();
}

void IcebusHost::SendStatusRequest(int motor){
  StatusRequest req;
  req.values.header = 0xBBCEE11C;
  req.values.motor_id = motor;
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
  msg.values.header = 0xD0D0D0D0;
  msg.values.motor_id = motor;
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
  printf("control_mode------------>\t");
  for(int i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  Serial1.write(msg.data, 28);
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
  printf("status_response------------>\t");
  for(int i=0;i<sizeof(msg);i++){
      printf("%x\t",msg.data[i]);
  }
  printf("\n");
  Serial1.write(msg.data, 28);
}

void IcebusHost::Listen(int motor){
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

    if(header==0x1CE1CEBB){
      crc crc_received = gen_crc16(&read_buf[4],n-4-2);
      if(crc_received==(read_buf[n-1]<<8|read_buf[n-2])){
        printf("status request received for motor_id %d\n", read_buf[4]);
        SendStatusResponse(motor);
      }else{
        printf("crc dont match, crc sent %x, crc calculated %x\n", (read_buf[6]<<8|read_buf[5]),crc_received);
      }
    }else if(header==0xD0D0D0D0){
      crc crc_received = gen_crc16(&read_buf[4],n-4-2);
      if(crc_received==(read_buf[n-1]<<8|read_buf[n-2])){
        printf("command received for motor_id %d\n", read_buf[4]);
      }else{
        printf("crc dont match, crc sent %x, crc calculated %x\n", (read_buf[6]<<8|read_buf[5]),crc_received);
      }
    }else if(header==0xBAADAA55){
      crc crc_received = gen_crc16(&read_buf[4],n-4-2);
      if(crc_received==(read_buf[n-1]<<8|read_buf[n-2])){
        printf("control_mode received for motor_id %d\n", read_buf[4]);
      }else{
        printf("crc dont match, crc sent %x, crc calculated %x\n", (read_buf[6]<<8|read_buf[5]),crc_received);
      }
    }else if(header==0x1CEB00DA){
      crc crc_received = gen_crc16(&read_buf[4],n-4-2);
      if(crc_received==(read_buf[n-1]<<8|read_buf[n-2])){
        printf("status response received for motor_id %d\n", read_buf[4]);
      }else{
        printf("crc dont match, crc sent %x, crc calculated %x\n", (read_buf[6]<<8|read_buf[5]),crc_received);
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
