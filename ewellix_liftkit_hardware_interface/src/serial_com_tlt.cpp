/* Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "liftkit_hardware_interface/serial_com_tlt.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

// hex values for ewellix specific params
constexpr int MOT1 = 0x00;
constexpr int MOT2 = 0x01;
constexpr unsigned char MOT1_ADDR = 0x11;
constexpr int MOT_ALL = 0x07;
constexpr unsigned char MOT2_ADDR = 0x12;
constexpr int UP = 0x02;
constexpr int DOWN = 0x01;
constexpr int UNUSED = 0Xff;
constexpr int SPEED_CMD = 0x04;
constexpr int SPEED_UNUSED = 0x00;
constexpr int REMOTE_DATA_ITEM = 0x30;
constexpr int OPEN_COMM = 0x01;

// If you command 0 for move speed, the lift will not treat it as a 0 speed,
// instead, we send a value of 2 which it will ifnore because it is below
// the threshold
constexpr int LIFT_CMD_NO_MOVE = 2;
// found that when moving up at 100 speed, we move at about 0.035 m/s
// which corresponds to a speed_cmd = desired_Speed * 3222
constexpr double FF_SCALE = 3222;
// Moving down needs a lot less effort for similar tracking because
// it has gravity working with it
constexpr double UP_TO_DOWN_SPEED_FACTOR = 0.45;
// controller gains
constexpr double Kp = 4000;
constexpr double Kd = 200.0;
constexpr double Ki = 4000.0;
constexpr double Ki_max = 10.0;
constexpr double Ki_min = -1.0 * Ki_max / UP_TO_DOWN_SPEED_FACTOR;
constexpr bool antiwindup = true;

// max value to command for either of the motors
constexpr int SPEED_HIGH_LIMIT = 100;  // As percentage of 100
// lower limit to command for the motors. If you command under this, the
// liftkit will not move, and it will end up in an error state until you
// force it to change directions, which can be a hassle.
std::vector<int> SPEED_LOW_LIMIT_UP = { 32, 32 };    // As percentage of 100
std::vector<int> SPEED_LOW_LIMIT_DOWN = { 29, 29 };  // As percentage of 100

SerialComTlt::SerialComTlt()
  : run_(true)
  , debug_(false)
  , stop_loop_(false)
  , com_started_(false)
  , first_time_(true)
  , height_limit_(0.7)
  , meters_to_ticks_(3222.65)
  , min_ticks_mot_1_(10)
  , max_ticks_mot_1_(800)
  , min_ticks_mot_2_(10)
  , max_ticks_mot_2_(800)
  , desired_pose_(std::numeric_limits<double>::quiet_NaN())
  , desired_velocity_(0.0)
  , current_pose_(0.0)
  , previous_pose_(0.0)
  , current_velocity_(0.0)
  , commanded_velocity_(0.0)
  , mot1_pose_(0)
  , mot2_pose_(0)
  , mot1_pose_m_(0)
  , mot2_pose_m_(0)
  , mot_ticks_(0)
  , lock_()
  , serial_tlt_()
  , pid_()
  , speed_commands_(2, LIFT_CMD_NO_MOVE)
  , curr_dirs_(2, DIR::MOVING_STOPPED)
  , last_dirs_(2, DIR::MOVING_STOPPED)
  , should_moves_(2, 0)
  , stoppeds_(2, false)
  , num_cycles_waiteds_(2, 0)
  , curr_dir(MOVING_STOPPED)
  , last_dir(MOVING_STOPPED)
  , cycles_to_wait(2)
{
}

SerialComTlt::~SerialComTlt()
{
  stop();
  com_started_ = false;
  stop_loop_ = true;
  run_ = false;
  if (serial_tlt_.isOpen())
  {
    serial_tlt_.close();
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "SerialComTlt::~SerialComTlt - COM CLOSED !");
  }
}

bool SerialComTlt::startSerialCom(string port, int baud_rate)
{
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);

  serial_tlt_.setPort(port);
  serial_tlt_.setBaudrate(baud_rate);
  serial_tlt_.setTimeout(timeout);

  try
  {
    serial_tlt_.open();
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "SerialComTlt::startSerialCom - COM OPEN !");
    return true;
  }
  catch (serial::IOException& e)
  {
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::startSerialCom - serial::IOException: %s", e.what());
    return false;
  }
}

/// Activate the remote function
bool SerialComTlt::startRs232Com()
{
  vector<unsigned char> params = { OPEN_COMM };
  sendCmd("RO", &params);
  bool result = sendCmd("RO", &params);  // double call to wake up the column
                                         // after long delay without com
  if (result)
  {
    com_started_ = true;
    stop_loop_ = true;
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::startRs232Com - Remote function activated");
    vector<unsigned char> params = { 0x01, 0x00, 0xff };  // Start cyclic communications
    sendCmd("RC", &params);
    getColumnSize();
    // send the lift to the home position
    // setColumnSize(0.0);
    pid_ = control_toolbox::Pid(Kp, Ki, Kd, Ki_max, Ki_min, antiwindup);
    pid_.reset();
    return true;
  }
  else
  {
    return false;
  }
}

/// Deactivate the remote function
bool SerialComTlt::stopRs232Com()
{
  com_started_ = false;
  std::this_thread::sleep_for(std::chrono::microseconds(100));
  vector<unsigned char> params = {};
  bool result = sendCmd("RA", &params);

  if (result)
  {
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::stopRs232Com - Remote function deactivated");
    return true;
  }
  else
  {
    return false;
  }
}

/// Move Mot1 with input pose (ntick)
void SerialComTlt::moveMot1Pose(int pose)
{
  vector<unsigned char> pose_hex = intToBytes(pose);
  unsigned char a = *(pose_hex.end() - 4);
  unsigned char b = *(pose_hex.end() - 3);
  unsigned char c = *(pose_hex.end() - 2);
  unsigned char d = *(pose_hex.end() - 1);

  vector<unsigned char> params = { 0x06, 0x00, 0x21, 0x30, d, c, b, a };

  sendCmd("RT", &params);  // set pose
  params = { 0x04, 0x00, 0x11, 0x30, 0x64, 0x00 };
  sendCmd("RT", &params);  // set speed 100%
  params = { 0x00, 0x09, 0xff };
  sendCmd("RE", &params);  // move
}

/// Move Mot2 with input pose (ntick)
void SerialComTlt::moveMot2Pose(int pose)
{
  vector<unsigned char> pose_hex = intToBytes(pose);
  unsigned char a = *(pose_hex.end() - 4);
  unsigned char b = *(pose_hex.end() - 3);
  unsigned char c = *(pose_hex.end() - 2);
  unsigned char d = *(pose_hex.end() - 1);

  vector<unsigned char> params = { 0x06, 0x00, 0x22, 0x30, d, c, b, a };

  sendCmd("RT", &params);  // set pose
  params = { 0x04, 0x00, 0x12, 0x30, 0x64, 0x00 };
  sendCmd("RT", &params);  // set speed 100%

  params = { 0x01, 0x09, 0xff };
  sendCmd("RE", &params);  // move
}

///  Move up both motors
void SerialComTlt::moveUp()
{
  moveMot1Up();
  moveMot2Up();
}

///  Move down both motors
void SerialComTlt::moveDown()
{
  moveMot1Down();
  moveMot2Down();
}

///  Move up Mot1
void SerialComTlt::moveMot1Up()
{
  vector<unsigned char> params = { MOT1, UP, UNUSED };
  sendCmd("RE", &params);
}

///  Move up Mot2
void SerialComTlt::moveMot2Up()
{
  vector<unsigned char> params = { MOT2, UP, UNUSED };
  sendCmd("RE", &params);
}

///  Move down Mot1
void SerialComTlt::moveMot1Down()
{
  vector<unsigned char> params = { MOT1, DOWN, UNUSED };
  sendCmd("RE", &params);
}

//  Move down Mot2
void SerialComTlt::moveMot2Down()
{
  vector<unsigned char> params = { MOT2, DOWN, UNUSED };
  sendCmd("RE", &params);
}

// Set the speed of both Mot1 and Mot2
void SerialComTlt::setLiftSpeed(int speed)
{
  speed = speed / 2;
  setLiftSpeedForMotor(speed, MOTOR_NUM::MOTOR_ONE);
  setLiftSpeedForMotor(speed, MOTOR_NUM::MOTOR_TWO);
}

// Set the speed of both Mot1 and Mot2
void SerialComTlt::setLiftSpeedForMotor(int speed, MOTOR_NUM motor_num)
{
  // Clamp max speed command
  if (abs(speed) > SPEED_HIGH_LIMIT)
    speed = SPEED_HIGH_LIMIT;

  // Explicitly convert speed for the command interface
  auto speed_param = static_cast<unsigned char>(abs(speed));

  auto mot_addr = (motor_num == MOTOR_NUM::MOTOR_ONE) ? MOT1_ADDR : MOT2_ADDR;

  // Set speed for desired motor
  vector<unsigned char> params = { SPEED_CMD, SPEED_UNUSED, mot_addr, REMOTE_DATA_ITEM, speed_param, SPEED_UNUSED };
  sendCmd("RT", &params);
}

/// Stop both motors
void SerialComTlt::stop()
{
  stopMot1();
  stopMot2();
}

/// Stop Mot1
void SerialComTlt::stopMot1()
{
  num_cycles_waiteds_[0] = 0;
  stoppeds_[0] = true;
  vector<unsigned char> params = { MOT1, 0x00 };  // 0 fast stop   1 smooth stop
  sendCmd("RS", &params);                         // stop moving
}

/// Stop Mot2
void SerialComTlt::stopMot2()
{
  num_cycles_waiteds_[1] = 0;
  stoppeds_[1] = true;
  vector<unsigned char> params = { MOT2, 0x00 };
  sendCmd("RS", &params);  // stop moving
}

/// Stop all motors
void SerialComTlt::stopMotAll()
{
  vector<unsigned char> params = { MOT_ALL, 0x00 };  // 0 fast stop   1 smooth stop
  sendCmd("RS", &params);                            // stop moving
}

/// Get position
double SerialComTlt::getColumnSize()
{
  getPoseM1();
  getPoseM2();
  previous_pose_ = current_pose_;

  int ticks_offset = (min_ticks_mot_1_ + min_ticks_mot_2_) - static_cast<int>(meters_to_ticks_ * min_height_m_);

  current_pose_ = double(mot1_pose_ + mot2_pose_ - ticks_offset) * 1.0 / meters_to_ticks_;

  return current_pose_;
}

/// Stop velocity
double SerialComTlt::getColumnVelocity()
{
  return current_velocity_;
}

/// Get raw position of Mot1
void SerialComTlt::getPoseM1()
{
  vector<unsigned char> params = { MOT1_ADDR, 0x00 };
  sendCmd("RG", &params);
}

/// Get raw position of Mot2
void SerialComTlt::getPoseM2()
{
  vector<unsigned char> params = { MOT2_ADDR, 0x00 };
  sendCmd("RG", &params);
}

/// Loop to maintain the remote function with RC command
void SerialComTlt::calibrationComLoop(std::string calibration_direction)
{
  vector<unsigned char> params = { 0x01, 0x00, 0xff };

  // run_ starts as true with class initialization and is true until the HWI kills the program
  while (run_)
  {
    // com_started_ is set to true on connection of comm, so we are good to actually do communication
    while (com_started_)
    {
      sendCmd("RC", &params);

      // get the position data from the liftkit for ROS
      getColumnSize();

      if (first_time_)
      {
        setLiftSpeed(100);
        if (calibration_direction == "up")
        {
          moveMot1Up();
          moveMot2Up();
        }
        else if (calibration_direction == "down")
        {
          moveMot1Down();
          moveMot2Down();
        }
      }
      else if (current_pose_ == previous_pose_)
      {
        if (calibration_counter++ > 10)
        {
          std::string min_max_string = calibration_direction == "up" ? "max" : "min";
          RCLCPP_INFO_ONCE(rclcpp::get_logger("LiftkitHardwareInterface"), "Calibration complete! Direction: %s",
                           calibration_direction.c_str());
          RCLCPP_INFO_ONCE(rclcpp::get_logger("LiftkitHardwareInterface"), "mot1_%s_ticks: %i", min_max_string.c_str(),
                           mot1_pose_);
          RCLCPP_INFO_ONCE(rclcpp::get_logger("LiftkitHardwareInterface"), "mot2_%s_ticks: %i", min_max_string.c_str(),
                           mot2_pose_);
        }
      }
      else
      {
        calibration_counter = 0;
      }

      // no longer the first loop
      first_time_ = false;

      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
}

/// Loop to maintain the remote function with RC command
void SerialComTlt::comLoop()
{
  vector<unsigned char> params = { 0x01, 0x00, 0xff };
  int speed_command;
  auto last_time = std::chrono::steady_clock::now();
  auto curr_time = std::chrono::steady_clock::now();

  // run_ starts as true with class initialization and is true until the HWI kills the program
  while (run_)
  {
    // com_started_ is set to true on connection of comm, so we are good to actually do communication
    while (com_started_)
    {
      sendCmd("RC", &params);

      // read portion

      // get period of cycle for velocity calculation
      curr_time = std::chrono::steady_clock::now();
      int64_t dt_read_ = std::chrono::duration_cast<std::chrono::nanoseconds>(curr_time - last_time).count();

      // get the position data from the liftkit for ROS
      getColumnSize();

      if (first_time_)
      {
        desired_pose_ = current_pose_;
      }

      // get error and threshold state
      double error = desired_pose_ - current_pose_;

      // calculate actual and desired velocity to pass back to ROS
      current_velocity_ = (current_pose_ - previous_pose_) / (static_cast<double>(dt_read_) / 1.0e9);

      // write portion

      // command speed based on a PID controller as well as FF value
      double ff_speed = desired_velocity_ * FF_SCALE;
      double fb_command = pid_.computeCommand(error, dt_read_);
      speed_command = static_cast<int>(fb_command + ff_speed);

      // The robot actually moves a lot faster going down for the same
      // speed - I estimated it needs about 70% of the command to move
      // at the same speed.
      if (speed_command < 0)
        speed_command = static_cast<int>(static_cast<double>(speed_command) * UP_TO_DOWN_SPEED_FACTOR);
      commanded_velocity_ = speed_command;

      // set the current moving direction based on
      if (speed_command > 0)
        curr_dirs_ = { MOVING_UP, MOVING_UP };
      else if (speed_command < 0)
        curr_dirs_ = { MOVING_DOWN, MOVING_DOWN };
      else
        curr_dirs_ = { MOVING_STOPPED, MOVING_STOPPED };

      // default initialize speed commands to 0s
      speed_commands_ = { 0, 0 };
      constexpr int max_height_epsilon = 10;  // number of ticks away from extreme that we switch motors
      // if we are moving up, and mot1 is < (0.35 - epsilon), use mot1
      // otherwise use mot2
      if (curr_dirs_[0] == MOVING_UP)
      {
        if (mot1_pose_ < max_ticks_mot_1_ - max_height_epsilon)
          speed_commands_[0] = speed_command;
        else
          speed_commands_[1] = speed_command;
      }
      // if we are moving down, and mot2 is < (0 + epsilon), use mot1
      // otherwise use mot2
      else if (curr_dirs_[0] == MOVING_DOWN)
      {
        if (mot2_pose_ < (min_ticks_mot_2_ + max_height_epsilon))
          speed_commands_[0] = speed_command;
        else
          speed_commands_[1] = speed_command;
      }

      // loop over each of the motors to perform the same logic of what state
      // they should be in
      std::vector<MOTOR_NUM> motor_nums = { MOTOR_NUM::MOTOR_ONE, MOTOR_NUM::MOTOR_TWO };
      for (const auto& motor : motor_nums)
      {
        size_t motor_index = (motor == MOTOR_ONE) ? 0 : 1;
        auto speed_limit_this_direction =
            (curr_dirs_[motor_index] == MOVING_UP) ? SPEED_LOW_LIMIT_UP : SPEED_LOW_LIMIT_DOWN;
        // if we are consistently moving in the same direction, we should move
        should_moves_[motor_index] = (curr_dirs_[motor_index] == last_dirs_[motor_index]) &&
                                     (abs(speed_commands_[motor_index]) > speed_limit_this_direction[motor_index]);

        // default initialize to no movement on motor i
        int lift_cmd_speed = LIFT_CMD_NO_MOVE;
        // if we said we should be moving, reassign to calculated cmd
        if (should_moves_[motor_index])
          lift_cmd_speed = speed_commands_[motor_index];

        // send command for whatever motor we are using.
        setLiftSpeedForMotor(lift_cmd_speed, motor);

        // we need to stop if we (1) have a should be stopped flag and we are
        // not currently stopped or (2) it is the first time through
        if ((!should_moves_[motor_index] && !stoppeds_[motor_index]) || first_time_)
        {
          if (motor == MOTOR_NUM::MOTOR_ONE)
            stopMot1();
          else
            stopMot2();
        }
        // if we are in a state where we should be moving (velocity cmd above
        // min threshold) and we are stopped, and we have waited the appropriate
        // number of cycles, tell the correct motor to move
        else if (should_moves_[motor_index] && stoppeds_[motor_index] &&
                 num_cycles_waiteds_[motor_index] >= cycles_to_wait)
        {
          // if speed is greater than 0, we need to be commanding a move up
          if (speed_commands_[motor_index] > 0)
          {
            if (motor == MOTOR_NUM::MOTOR_ONE)
              moveMot1Up();
            else
              moveMot2Up();
          }
          // if speed is less than 0, we need to be commanding a move down
          else
          {
            if (motor == MOTOR_NUM::MOTOR_ONE)
              moveMot1Down();
            else
              moveMot2Down();
          }
          stoppeds_[motor_index] = false;
        }

        // we always increment number of cycles waited. Stopping
        // resets this counter, so
        num_cycles_waiteds_[motor_index]++;
      }

      // no longer the first loop
      first_time_ = false;

      // update persistent values for next iteration
      last_dirs_ = curr_dirs_;
      last_time = curr_time;

      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
}

/// Convert the command in bytes and compute the checksum before writing to
/// serial com
bool SerialComTlt::sendCmd(string cmd, vector<unsigned char>* param)
{
  lock_.lock();
  vector<unsigned char> final_cmd;
  for (const auto& item : cmd)
  {
    final_cmd.push_back(int(item));  // convert cmd string in hex value
  }

  if (!param->empty())
  {
    for (vector<unsigned char>::iterator it = param->begin(); it != param->end(); ++it)
    {
      final_cmd.push_back(*it);  // add params to the cmd
    }
  }
  // Compute checksum
  unsigned short checksum = calculateChecksum(&final_cmd);
  unsigned short lsb = checksum & 0x00FF;
  unsigned short msb = checksum >> 8;

  final_cmd.push_back(lsb);
  final_cmd.push_back(msb);

  if (debug_)
  {
    stringstream final_cmd_hex;
    for (unsigned short j : final_cmd)
    {
      final_cmd_hex << hex << j << ' ';
    }
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "SerialComTlt::sendCmd - Output Cmd: %s",
                final_cmd_hex.str().c_str());
  }

  if (serial_tlt_.isOpen())
  {
    try
    {
      serial_tlt_.write(final_cmd);
      std::this_thread::sleep_for(std::chrono::microseconds(1));
      serial_tlt_.flush();
      stop_loop_ = false;
    }
    catch (serial::IOException& e)
    {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "SerialComTlt::sendCmd - Output Cmd: %s", e.what());
    }
  }
  std::this_thread::sleep_for(std::chrono::microseconds(10));

  vector<unsigned char> output = feedback();
  if (output.size() == 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "SerialComTlt::sendCmd - Response empty");
    lock_.unlock();
    return false;
  }
  if (debug_)
  {
    stringstream output_hex;
    for (unsigned short j : output)
    {
      output_hex << hex << j << ' ';
    }
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "SerialComTlt::sendCmd - Response: %s",
                output_hex.str().c_str());
  }

  if (!checkResponseChecksum(&output) || !checkResponseAck(&output))
  {
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "SerialComTlt::sendCmd - Cmd failed, retry");
    serial_tlt_.flush();
    std::this_thread::sleep_for(std::chrono::microseconds(300));
    serial_tlt_.write(final_cmd);

    output = feedback();
    if (debug_)
    {
      stringstream output_hex;
      for (unsigned short j : output)
      {
        output_hex << hex << j << ' ';
      }
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "SerialComTlt::sendCmd - Response: %s",
                  output_hex.str().c_str());
    }
    if (output.size() == 0 || !checkResponseChecksum(&output) || !checkResponseAck(&output))
    {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "SerialComTlt::sendCmd - Command FAILED !!!");
      lock_.unlock();
      startRs232Com();
      return false;
    }
  }

  if (cmd == "RG")
  {
    if (*(param->begin()) == 0x11)
      extractPose(&output, 1);
    else if (*(param->begin()) == 0x12)
      extractPose(&output, 2);
  }

  lock_.unlock();
  return true;
}

vector<unsigned char> SerialComTlt::feedback()
{
  int i = 0;
  int timeout = 0;
  vector<unsigned char> received_data;
  string last_data;
  string command_type = "";
  int msg_size = -1;
  bool success = false;
  bool first_byte = false;

  while (!stop_loop_)
  {
    // Check if serial available
    if (serial_tlt_.available())
    {
      last_data = serial_tlt_.read();

      if (!first_byte && last_data == "R")
      {  // Beginning of the message
        first_byte = true;
        for (const auto& item : last_data)
        {
          received_data.push_back(int(item));  // convert cmd string in hex value
        }
      }
      if (first_byte)
      {
        i++;
        if (i > 1)
        {
          for (const auto& item : last_data)
          {
            received_data.push_back(int(item));  // convert cmd string in hex value
          }
        }

        // command detector
        if (i == 2)
        {
          command_type = last_data;
        }

        // success detector
        if (i == 3 && last_data == "")
        {  // ACK
          success = true;
        }

        if (command_type == "G")
        {
          if (i == 4 && success)
          {
            for (const auto& item : last_data)
            {
              msg_size = 7 + int(item);
            }
          }
          else if (i == 4 && !success)
          {
            msg_size = 5;
          }
        }
        else if (command_type == "T" || command_type == "C" || command_type == "E" || command_type == "S" ||
                 command_type == "O" || command_type == "A")
        {
          msg_size = 5;
        }
        else
        {
          msg_size = 5;
        }

        if (msg_size > 0 && i == msg_size)
        {
          stop_loop_ = true;
          break;
        }
      }
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::microseconds(1));
      timeout++;
      if (timeout > 5000)
      {
        stop_loop_ = true;
        return received_data;
      }
    }
  }
  return received_data;
}

/// Compute the checksum
unsigned short SerialComTlt::calculateChecksum(vector<unsigned char>* cmd)
{
  unsigned short crc = 0;
  for (vector<unsigned char>::iterator it = cmd->begin(); it != cmd->end(); ++it)
  {
    crc = static_cast<unsigned short>(CRC_TABLE[((crc >> 8) ^ *it) & 0xFF] ^ (crc << 8));
  }
  return crc;
}

/// Compare the response checksum with the calculated
bool SerialComTlt::checkResponseChecksum(vector<unsigned char>* response)
{
  unsigned short response_checksum_lsb = *(response->end() - 2);
  unsigned short response_checksum_msb = *(response->end() - 1);

  if (debug_)
  {
    unsigned short checksum = (response_checksum_lsb << 8) | response_checksum_msb;
    stringstream checksum_hex;
    checksum_hex << hex << checksum;
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::checkResponseChecksum - Response Checksum = %s", checksum_hex.str().c_str());
  }

  vector<unsigned char> response_msg = *response;
  response_msg.resize(response_msg.size() - 2);

  unsigned short checksum = calculateChecksum(&response_msg);
  unsigned short computed_lsb = checksum & 0x00FF;
  unsigned short computed_msb = checksum >> 8;

  if (debug_)
  {
    unsigned short checksum2 = (computed_lsb << 8) | computed_msb;
    stringstream checksum_hex;
    checksum_hex << hex << checksum2;
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::checkResponseChecksum - Computed Checksum = %s", checksum_hex.str().c_str());
  }

  if (response_checksum_lsb == computed_lsb && response_checksum_msb == computed_msb)
  {
    return true;
  }
  else
  {
    return false;
  }

  return false;
}

bool SerialComTlt::checkResponseAck(vector<unsigned char>* response)
{
  if (response->size() > 4)
  {
    unsigned short cmd_status = *(response->begin() + 2);
    if (cmd_status == 0x06)
    {
      return true;
    }

    else if (cmd_status == 0x81)
    {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                  "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 81 "
                  "Parameter data error ");
    }
    else if (cmd_status == 0x82)
    {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                  "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 82 "
                  "Parameter count error");
    }
    else if (cmd_status == 0x83)
    {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                  "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 83 "
                  "Command error ");
    }
    else if (cmd_status == 0x84)
    {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                  "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 84 "
                  "Permission error ");
    }
  }
  else
  {
    return false;
  }
  return false;
}

///  Extract motor pose from the response
bool SerialComTlt::extractPose(vector<unsigned char>* response, int motor)
{
  int position = (unsigned char)(*(response->end() - 5)) << 8 | (unsigned char)(*(response->end() - 6));

  if (motor == 1)
    mot1_pose_ = position;
  else if (motor == 2)
    mot2_pose_ = position;
  else
    return false;

  return true;
}

/// Converter
vector<unsigned char> SerialComTlt::intToBytes(int paramInt)
{
  vector<unsigned char> arrayOfByte(4);
  for (int i = 0; i < 4; i++)
    arrayOfByte[3 - i] = (paramInt >> (i * 8));
  return arrayOfByte;
}
