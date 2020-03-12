
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
        uint8_t motor_id;
        uint16_t crc;
    }values;
    uint8_t data[7];
};

union StatusResponse{
  struct __attribute__((packed)) {
        uint32_t header;
        uint8_t motor_id;
        uint8_t control_mode;
        int32_t encoder_position0:24;
        int32_t encoder_position1:24;
        int32_t setpoint:24;
        int32_t duty:24;
        int32_t displacement:24;
        int16_t current;
        int32_t neopixel_color:24;
        uint16_t crc;
    }values;
    uint8_t data[28];
};

union Command{
  struct __attribute__((packed)) {
        uint32_t header;
        uint8_t motor_id;
        int32_t setpoint:24;
        int32_t neopxl_color:24;
        uint16_t crc;
    }values;
    uint8_t data[13];
};

union ControlMode{
  struct __attribute__((packed)) {
        uint32_t header;
        uint8_t motor_id;
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
    }values;
    uint8_t data[28];
};

class IcebusHost{
public:
  IcebusHost();
  void SendStatusRequest(int motor);
  void SendStatusResponse(int motor);
  void SendCommand(int motor);
  void SendControlMode(int motor);
  void Listen(int motor);
private:
  crc  crcTable[256];
  void crcInit();
  crc gen_crc16(const uint8_t *data, uint16_t size);
};
