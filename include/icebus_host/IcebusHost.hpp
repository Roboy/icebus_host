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
    uart termios usage based mainly on: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
*/


#pragma once

#include <ros/ros.h>
#include <thread>
#include <std_srvs/SetBool.h>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/MotorConfig.hpp>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorInfo.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <roboy_middleware_msgs/MotorState.h>

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
        uint16_t setpoint0;
        uint16_t setpoint1;
        uint16_t setpoint2;
        uint16_t setpoint3;
        uint16_t crc;
    }values = {.header = 0x0DF005B1};
    uint8_t data[15];
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
        uint8_t control_mode;
        uint16_t setpoint0;
        uint16_t setpoint1;
        uint16_t setpoint2;
        uint16_t setpoint3;
        uint16_t position0;
        uint16_t position1;
        uint16_t position2;
        uint16_t position3;
        uint16_t current0;
        uint16_t current1;
        uint16_t current2;
        uint16_t current3;
        int32_t neopixel_color:24;
        uint16_t crc;
    }values = {.header = 0x35B1000B };
    uint8_t data[35];
};

union M3Command{
    struct __attribute__((packed)) {
          uint32_t header;
          uint8_t id;
          int32_t setpoint;
          uint16_t crc;
      }values = {.header = 0xBEBAFECA };
      uint8_t data[11];
  };

  union M3ControlMode{
    struct __attribute__((packed)) {
          uint32_t header;
          uint8_t id;
          uint8_t control_mode;
          uint16_t crc;
      }values = {.header = 0x0DD0FECA };
      uint8_t data[8];
  };

  union M3StatusResponse{
    struct __attribute__((packed)) {
          uint32_t header;
          uint8_t id;
          uint8_t control_mode;
          int32_t setpoint;
          int32_t pos;
          int32_t vel;
          int32_t dis;
          int32_t pwm;
          uint16_t crc;
      }values = {.header = 0x00D0BADA };
      uint8_t data[28];
  };

class IcebusHost{
public:
  IcebusHost(string device = "/dev/ttyUSB0", string motor_config_file_path = "roboy3.yaml");
  ~IcebusHost(){
    close(serial_port);
  }
  void SendStatusRequest(int id);
  void SendStatusResponse(int id);
  void SendCommand(int id);
  void SendControlMode(int id);
  void SendHandCommand(int id, vector<uint16_t> pos);
  void SendHandControlMode(int id);
  void SendHandStatusRequest(int id);
  void SendHandStatusResponse(int id);
  void SendM3Command(int id, int32_t setpoint);
  void SendM3ControlMode(int id, uint8_t control_mode);
  void Listen(int id);
  /**
   * Publishes information about motors
   */
  void MotorInfoPublisher();

  /**
   * Publishes state of motors
   */
  void MotorStatePublisher();
  /**
   * Service for changing motor PID parameters
   * @param req PID parameters
   * @param res success
   * @return success
   */
  bool MotorConfigService(roboy_middleware_msgs::MotorConfigService::Request &req,
                          roboy_middleware_msgs::MotorConfigService::Response &res);
  /**
   * Callback for motor command
   * @param msg motor command
   */
  void MotorCommand(const roboy_middleware_msgs::MotorCommand::ConstPtr &msg);
  /**
   * Emergency stop service, zeros all PID gains, causing all motors to stop, PID parameters and control mode are restored on release
   * @param req
   * @param res
   * @return
   */
  bool EmergencyStopService(std_srvs::SetBool::Request &req,
                            std_srvs::SetBool::Response &res);
  /**
   * Service for changing the control mode of motors, perviously set PID parameters are restored
   * @param req control mode
   * @param res
   * @return success
   */
  bool ControlModeService(roboy_middleware_msgs::ControlMode::Request &req,
                          roboy_middleware_msgs::ControlMode::Response &res);

  int32_t interpret24bitAsInt32(uint8_t *byteArray) {
      return (
          (byteArray[0] << 24)
      |   (byteArray[1] << 16)
      |   (byteArray[2] << 8)
      ) >> 8;
  };

  MotorConfigPtr motor_config;
private:
  ros::NodeHandlePtr nh;
  boost::shared_ptr<ros::AsyncSpinner> spinner;
  ros::Subscriber motorCommand_sub;
  ros::Publisher motorState, motorInfo;
  bool keep_publishing = true, emergency_stop = false;
  ros::ServiceServer motorConfig_srv, controlMode_srv, emergencyStop_srv;
  boost::shared_ptr<std::thread> motorInfoThread, motorStateThread;
  map<int, float> setpoint, encoder0_pos, encoder1_pos, displacement, current;
  map<int, float> Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit, current_limit, pwm;
  map<int, float> communication_quality;
  map<int, map<int, control_Parameters_t>> control_params_backup;
  map<int, int> control_mode_backup,control_mode;

  crc  crcTable[256];
  void crcInit();
  crc gen_crc16(const uint8_t *data, uint16_t size);
  int serial_port;
  struct termios tty;
};
