/*
    BSD 3-Clause License
    Copyright (c) 2020, Simon Trendel
            All rights reserved.
    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    author: Simon Trendel ( st@gi.ai ), 2020
    description: class for controlling icebus
*/

#pragma once

#include <ros/ros.h>
// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

using namespace std;
#pragma pack(1)
/*
 * The width of the CRC calculation and result.
 * Modify the typedef for a 16 or 32-bit CRC standard.
 */
typedef uint16_t crc;

#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))
#define POLYNOMIAL 0x8005

union StatusRequest{
  struct{
        uint32_t header;
        uint8_t motor_id;
        uint16_t crc;
    }values;
    uint8_t data[7];
};

union StatusResponse{
  struct{
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
  struct{
        uint32_t header;
        uint8_t motor_id;
        int32_t setpoint:24;
        int32_t neopxl_color:24;
        uint16_t crc;
    }values;
    uint8_t data[13];
};

union ControlMode{
  struct{
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
  IcebusHost(string device = "/dev/ttyUSB0");
  ~IcebusHost(){
    close(serial_port);
  }
  void SendStatusRequest(int motor);
  void SendStatusResponse(int motor);
  void SendCommand(int motor);
  void SendControlMode(int motor);
  void Listen(int motor);
private:
  crc  crcTable[256];
  void crcInit();
  crc gen_crc16(const uint8_t *data, uint16_t size);
  int serial_port;
  struct termios tty;
};
