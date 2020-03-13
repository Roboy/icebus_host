
#pragma pack(1)
/*
 * The width of the CRC calculation and result.
 * Modify the typedef for a 16 or 32-bit CRC standard.
 */
#include <Arduino.h>
#include <stdio.h>
typedef uint16_t crc;

#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))
#define POLYNOMIAL 0x8005

union StatusRequest{
  struct __attribute__((packed)) {
        uint32_t header;
        uint8_t id;
        uint16_t crc;
    }values = {.header = 0xBBCEE11C};
    uint8_t data[7];
};

union StatusResponse{
  struct __attribute__((packed)) {
        uint32_t header;
        uint8_t id;
        uint8_t control_mode;
        int32_t encoder_position0:24;
        int32_t encoder_position1:24;
        int32_t setpoint:24;
        int32_t duty:24;
        int32_t displacement:24;
        int16_t current;
        int32_t neopixel_color:24;
        uint16_t crc;
    }values = {.header = 0xDA00EB1C};
    uint8_t data[28];
};

union Command{
  struct __attribute__((packed)) {
        uint32_t header;
        uint8_t id;
        int32_t setpoint:24;
        int32_t neopxl_color:24;
        uint16_t crc;
    }values = {.header = 0xD0D0D0D0};
    uint8_t data[13];
};

union ControlMode{
  struct __attribute__((packed)) {
        uint32_t header;
        uint8_t id;
        uint8_t control_mode;
        int16_t Kp;
        int16_t Ki;
        int16_t Kd;
        int32_t PWMLimit:24;
        int32_t IntegralLimit:24;
        int32_t deadband:24;
        int32_t setpoint:24;
        int16_t current_limit;
        uint16_t crc;
    }values = {.header = 0x55AAADBA};
    uint8_t data[28];
};

union HandStatusRequest{
  struct __attribute__((packed)) {
        uint32_t header;
        uint8_t id;
        uint16_t crc;
    }values = {.header = 0xBEBAADAB};
    uint8_t data[7];
};

union HandCommand{
  struct __attribute__((packed)) {
        uint32_t header;
        uint8_t id;
        uint32_t setpoint;
        uint32_t neopxl_color:24;
        uint16_t crc;
    }values = {.header = 0x0DF005B1};
    uint8_t data[14];
};

union HandControlMode{
  struct __attribute__((packed)) {
        uint32_t header;
        uint8_t id;
        uint32_t control_mode;
        uint16_t crc;
    }values = {.header= 0xB5006BB1 };
    uint8_t data[11];
};

union HandStatusResponse{
  struct __attribute__((packed)) {
        uint32_t header;
        uint8_t id;
        uint32_t control_mode;
        uint32_t setpoint;
        uint32_t position;
        uint32_t current;
        int32_t neopixel_color:24;
        uint16_t crc;
    }values = {.header = 0x35B1000B };
    uint8_t data[22];
};

class IcebusHost{
public:
  IcebusHost();
  void SendStatusRequest(int id);
  void SendStatusResponse(int id);
  void SendCommand(int id);
  void SendControlMode(int id);
  void SendHandCommand(int id);
  void SendHandControlMode(int id);
  void SendHandStatusRequest(int id);
  void SendHandStatusResponse(int id);
  void Listen(int id);
private:
  crc  crcTable[256];
  void crcInit();
  crc gen_crc16(const uint8_t *data, uint16_t size);
};
