#include "liftkit_hardware_interface/serial_com_tlt.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

// error from desired under which we do not command the liftkit
// to move
constexpr double LIFTKIT_SETPOINT_THRESHOLD = 0.011;

// conversion values
constexpr double METERS_TO_TICKS = 1611.320754717;
constexpr double TICKS_TO_METERS = 0.000310304;

// offset value to
constexpr int TICKS_OFFSET = 10;

// hex values for ewellix specific params
constexpr int MOT1 = 0x00;
constexpr int MOT2 = 0x01;
constexpr int MOT1_ADDR = 0x11;
constexpr int MOT2_ADDR = 0x12;
constexpr int MOT_ALL = 0x07;
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
// Moving down needs 70% of the effort because it has gravity working with it
constexpr double UP_TO_DOWN_SPEED_FACTOR = 0.7; 
// feedback gain on the position error
constexpr double Kp = 4000;

constexpr int SPEED_HIGH_LIMIT = 100; // As percentage of 100
constexpr int SPEED_LOW_LIMIT = 32;   // As percentage of 100

SerialComTlt::SerialComTlt() : desired_vel_ema_(0.9) {
  run_ = true;
  debug_ = false;
  stop_loop_ = false;
  com_started_ = false;
  already_has_goal_ = false;
  mot1_pose_ = 0;
  mot2_pose_ = 0;
  mot_ticks_ = 0;

  last_target_ = 0.0;
  previous_pose_ = 0.0;
  current_pose_ = 0.0;
  current_target_ = std::numeric_limits<double>::quiet_NaN();;
  height_limit_ = 0.7;

  curr_dir = MOVING_STOPPED;
  last_dir = MOVING_STOPPED;

  speed_commands_ = std::vector<double>(2,LIFT_CMD_NO_MOVE);
  curr_dirs_ = std::vector<DIR>(2,DIR::MOVING_STOPPED);
  last_dirs_ = std::vector<DIR>(2,DIR::MOVING_STOPPED);
  should_moves_ = std::vector<bool>(2,0);
  stoppeds_ = std::vector<bool>(2,false);
  num_cycles_waiteds_ = std::vector<int>(2,0);
}

SerialComTlt::~SerialComTlt() {
  stop();
  com_started_ = false;
  stop_loop_ = true;
  run_ = false;
  if (serial_tlt_.isOpen()) {
    serial_tlt_.close();
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::~SerialComTlt - COM CLOSED !");
  }
}

bool SerialComTlt::startSerialCom(string port, int baud_rate) {

  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);

  serial_tlt_.setPort(port);
  serial_tlt_.setBaudrate(baud_rate);
  serial_tlt_.setTimeout(timeout);

  try {
    serial_tlt_.open();
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::startSerialCom - COM OPEN !");
    return true;
  } catch (serial::IOException &e) {
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::startSerialCom - serial::IOException: %s",
                e.what());
    return false;
  }
}

/// Activate the remote function
bool SerialComTlt::startRs232Com() {

  vector<unsigned char> params = {OPEN_COMM};
  sendCmd("RO", &params);
  bool result = sendCmd("RO", &params); // double call to wake up the column
                                        // after long delay without com
  if (result) {
    com_started_ = true;
    stop_loop_ = true;
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::startRs232Com - Remote function activated");
    vector<unsigned char> params = {0x01, 0x00,
                                    0xff}; // Start cyclic communications
    sendCmd("RC", &params);
    getColumnSize();
    // send the lift to the home position
    // setColumnSize(0.0);
    return true;
  } else {
    return false;
  }
}

/// Deactivate the remote function
bool SerialComTlt::stopRs232Com() {
  com_started_ = false;
  usleep(100);
  vector<unsigned char> params = {};
  bool result = sendCmd("RA", &params);

  if (result) {
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::stopRs232Com - Remote function deactivated");
    return true;
  } else {
    return false;
  }
}

/// Move Mot1 with input pose (ntick)
void SerialComTlt::moveMot1Pose(int pose) {
  vector<unsigned char> pose_hex = intToBytes(pose);
  unsigned char a = *(pose_hex.end() - 4);
  unsigned char b = *(pose_hex.end() - 3);
  unsigned char c = *(pose_hex.end() - 2);
  unsigned char d = *(pose_hex.end() - 1);

  vector<unsigned char> params = {0x06, 0x00, 0x21, 0x30, d, c, b, a};

  sendCmd("RT", &params); // set pose
  params = {0x04, 0x00, 0x11, 0x30, 0x64, 0x00};
  sendCmd("RT", &params); // set speed 100%
  params = {0x00, 0x09, 0xff};
  sendCmd("RE", &params); // move
}

/// Move Mot2 with input pose (ntick)
void SerialComTlt::moveMot2Pose(int pose) {
  vector<unsigned char> pose_hex = intToBytes(pose);
  unsigned char a = *(pose_hex.end() - 4);
  unsigned char b = *(pose_hex.end() - 3);
  unsigned char c = *(pose_hex.end() - 2);
  unsigned char d = *(pose_hex.end() - 1);

  vector<unsigned char> params = {0x06, 0x00, 0x22, 0x30, d, c, b, a};

  sendCmd("RT", &params); // set pose
  params = {0x04, 0x00, 0x12, 0x30, 0x64, 0x00};
  sendCmd("RT", &params); // set speed 100%

  params = {0x01, 0x09, 0xff};
  sendCmd("RE", &params); // move
}

/// Control the column size
void SerialComTlt::setColumnSize(double height) {
  if (height > height_limit_) {
    height = height_limit_;
  }

  if (getColumnSize() == height)
    return; // current pose = desired pose

  mot_ticks_ = (height * METERS_TO_TICKS) + TICKS_OFFSET;
  if (height <= 0)
    mot_ticks_ = TICKS_OFFSET; // min
  stop();
  moveMot1Pose(mot_ticks_);
  moveMot2Pose(mot_ticks_);
}

///  Move up both motors
void SerialComTlt::moveUp() {
  moveMot1Up();
  moveMot2Up();
}

///  Move down both motors
void SerialComTlt::moveDown() {
  moveMot1Down();
  moveMot2Down();
}

///  Move up Mot1
void SerialComTlt::moveMot1Up() {
  RCLCPP_INFO(rclcpp::get_logger("liftkit"), "motor 1 up");      
  vector<unsigned char> params = {MOT1, UP, UNUSED};
  sendCmd("RE", &params);
}

///  Move up Mot2
void SerialComTlt::moveMot2Up() {
  RCLCPP_INFO(rclcpp::get_logger("liftkit"), "motor 2 up");      
  vector<unsigned char> params = {MOT2, UP, UNUSED};
  sendCmd("RE", &params);
}

///  Move down Mot1
void SerialComTlt::moveMot1Down() {
  RCLCPP_INFO(rclcpp::get_logger("liftkit"), "motor 1 down");      
  vector<unsigned char> params = {MOT1, DOWN, UNUSED};
  sendCmd("RE", &params);
}

//  Move down Mot2
void SerialComTlt::moveMot2Down() {
  RCLCPP_INFO(rclcpp::get_logger("liftkit"), "motor 2 down");      
  vector<unsigned char> params = {MOT2, DOWN, UNUSED};
  sendCmd("RE", &params);
}

// Set the speed of both Mot1 and Mot2
void SerialComTlt::setLiftSpeed(int speed) {

  speed = speed / 2;
  setLiftSpeedForMotor(speed, MOTOR_NUM::MOTOR_ONE);
  setLiftSpeedForMotor(speed, MOTOR_NUM::MOTOR_TWO);
}

// Set the speed of both Mot1 and Mot2
void SerialComTlt::setLiftSpeedForMotor(int speed, MOTOR_NUM motor_num) {

  // Clamp max speed command
  if (abs(speed) > SPEED_HIGH_LIMIT)
    speed = SPEED_HIGH_LIMIT;

  // Explicitly convert speed for the command interface
  auto speed_param = static_cast<unsigned char>(abs(speed));

  auto mot_addr = (motor_num == MOTOR_NUM::MOTOR_ONE) ? MOT1_ADDR : MOT2_ADDR;

  // Set speed for desired motor
  vector<unsigned char> params = {SPEED_CMD,        SPEED_UNUSED, mot_addr,
                                  REMOTE_DATA_ITEM, speed_param,  SPEED_UNUSED};
  sendCmd("RT", &params);
}

/// Stop both motors
void SerialComTlt::stop() {  
  stopMot1();
  stopMot2();
}

/// Stop Mot1
void SerialComTlt::stopMot1() {
  RCLCPP_INFO(rclcpp::get_logger("liftkit"), "stopping motor 1");      
  num_cycles_waiteds_[0] = 0;
  stoppeds_[0] = true;
  vector<unsigned char> params = {MOT1, 0x00}; // 0 fast stop   1 smooth stop
  sendCmd("RS", &params);                      // stop moving
}

/// Stop Mot2
void SerialComTlt::stopMot2() {
  RCLCPP_INFO(rclcpp::get_logger("liftkit"), "stopping motor 2");      
  num_cycles_waiteds_[1] = 0;
  stoppeds_[1] = true;
  vector<unsigned char> params = {MOT2, 0x00};
  sendCmd("RS", &params); // stop moving
}

/// Stop all motors
void SerialComTlt::stopMotAll() {
  vector<unsigned char> params = {MOT_ALL, 0x00}; // 0 fast stop   1 smooth stop
  sendCmd("RS", &params);                         // stop moving
}

/// Get position
double SerialComTlt::getColumnSize() {
  getPoseM1();
  getPoseM2();
  previous_pose_ = current_pose_;
  current_pose_ =
      double(mot1_pose_ + mot2_pose_ - 2 * TICKS_OFFSET) * TICKS_TO_METERS;

  mot1_pose_m_ = double(mot1_pose_ - TICKS_OFFSET)*TICKS_TO_METERS;
  mot2_pose_m_ = double(mot2_pose_ - TICKS_OFFSET)*TICKS_TO_METERS;

  return current_pose_;
}

/// Stop velocity
double SerialComTlt::getColumnVelocity() { return current_velocity_; }

/// Get raw position of Mot1
void SerialComTlt::getPoseM1() {
  vector<unsigned char> params = {MOT1_ADDR, 0x00};
  sendCmd("RG", &params);
}

/// Get raw position of Mot2
void SerialComTlt::getPoseM2() {
  vector<unsigned char> params = {MOT2_ADDR, 0x00};
  sendCmd("RG", &params);
}

/// Loop to maintain the remote function with RC command
void SerialComTlt::comLoop() {
  vector<unsigned char> params = {0x01, 0x00, 0xff};
  int speed_command;
  auto last_time = std::chrono::steady_clock::now();
  auto curr_time = std::chrono::steady_clock::now();

  // run this while ROS has told us we are good to go
  while (run_) {
    while (com_started_) {
      sendCmd("RC", &params);

      // read portion

      // get period of cycle for velocity calculation
      curr_time = std::chrono::steady_clock::now();
      int64_t dt_read_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             curr_time - last_time)
                             .count();

      // get the position data from the liftkit for ROS
      getColumnSize();

      static std::vector<bool> first_times = {true, true};
      if (first_times[0]){
        current_target_ = current_pose_;
      }

      // get error and threshold state
      double error = current_target_ - current_pose_;
      // bool below_threshold = abs(error) < LIFTKIT_SETPOINT_THRESHOLD;

      desired_vel_ema_.add_value((current_target_ - last_target_) /
                                 (static_cast<double>(dt_read_) / 1.0e9));
      // desired_velocity_ = desired_vel_ema_.get_average();
      desired_velocity_ = (current_target_ - last_target_) /
                                 (static_cast<double>(dt_read_) / 1.0e9);

      // calculate actual and desired velocity to pass back to ROS
      current_velocity_ = (current_pose_ - previous_pose_) /
                          (static_cast<double>(dt_read_) / 1.0e9);


      // write portion

      // command speed based on a Proportional controller
      double ff_speed = desired_velocity_ * FF_SCALE;
      speed_command = static_cast<int>(Kp * error + ff_speed);
      if (speed_command < 0) speed_command *= UP_TO_DOWN_SPEED_FACTOR;
      commanded_velocity_ = speed_command;

      constexpr double max_height_epsilon = 0.01;
      if (mot1_pose_m_ < (height_limit_/2 - max_height_epsilon))
        speed_commands_[0] = speed_command;
      else
        speed_commands_[1] = speed_command;

      for (size_t i = 0; i < 2; i++)
      {
        // get the current direction we are moving. Ignore stop because we just
        // care about change in up->down
        if (speed_commands_[i] < 0.0)
          curr_dirs_[i] = MOVING_DOWN;
        else if (speed_commands_[i] > 0.0)
          curr_dirs_[i] = MOVING_UP;
        else
          curr_dirs_[i] = MOVING_STOPPED;

        // if we are consistently moving in the same direction, we should move
        should_moves_[i] = (curr_dirs_[i] == last_dirs_[i]) &&
                          ((abs(speed_commands_[i])) > SPEED_LOW_LIMIT);

        int lift_cmd_speed = LIFT_CMD_NO_MOVE; // do not move
        if (should_moves_[i]) lift_cmd_speed = speed_commands_[i];
        if (i == 0)
          setLiftSpeedForMotor(lift_cmd_speed,MOTOR_NUM::MOTOR_ONE);
        else
          setLiftSpeedForMotor(lift_cmd_speed,MOTOR_NUM::MOTOR_TWO);

        // if we are in a state where would should 
        if (should_moves_[i] && stoppeds_[i]){
          if (num_cycles_waiteds_[i] >= cycles_to_wait){
            if (speed_commands_[i] > 0) {
              if (i == 0)
                moveMot1Up();
              else
                moveMot2Up();
            } else {
              if (i == 0)
                moveMot1Down();
              else
                moveMot2Down();
            }
            stoppeds_[i] = false;
          }
        }
        else if (!should_moves_[i] && !stoppeds_[i] || first_times[i]){
          if (i == 0)
            stopMot1();
          else
            stopMot2();
        }

        //
        if (stoppeds_[i]){
          if (num_cycles_waiteds_[i]++ >= cycles_to_wait){
            if (i == 0)
              stopMot1();
            else
              stopMot2();
          }
        }
      }

      RCLCPP_INFO(rclcpp::get_logger("liftkit"), "speeds: [%0.3f, %0.3f]",speed_commands_[0], speed_commands_[1]);      
      
      first_times = {false, false};

      // update persistent values for next iteration
      last_dirs_ = curr_dirs_;
      last_target_ = current_target_;
      last_time = curr_time;

      usleep(100);
    }
    usleep(1);
  }
}

/// Convert the command in bytes and compute the checksum before writing to
/// serial com
bool SerialComTlt::sendCmd(string cmd, vector<unsigned char> *param) {
  lock_.lock();
  vector<unsigned char> final_cmd;
  for (const auto &item : cmd) {
    final_cmd.push_back(int(item)); // convert cmd string in hex value
  }

  if (!param->empty()) {
    for (vector<unsigned char>::iterator it = param->begin();
         it != param->end(); ++it) {
      final_cmd.push_back(*it); // add params to the cmd
    }
  }

  // Compute checksum
  unsigned short checksum = calculateChecksum(&final_cmd);
  unsigned short lsb = checksum & 0x00FF;
  unsigned short msb = checksum >> 8;

  final_cmd.push_back(lsb);
  final_cmd.push_back(msb);

  if (debug_) {

    stringstream final_cmd_hex;
    for (unsigned short j : final_cmd) {
      final_cmd_hex << hex << j << ' ';
    }
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::sendCmd - Output Cmd: %s",
                final_cmd_hex.str().c_str());
  }

  if (serial_tlt_.isOpen()) {
    try {
      serial_tlt_.write(final_cmd);
      usleep(1);
      serial_tlt_.flush();
      stop_loop_ = false;
    } catch (serial::IOException &e) {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                  "SerialComTlt::sendCmd - Output Cmd: %s", e.what());
    }
  }
  usleep(10);

  vector<unsigned char> output = feedback();
  if (output.size() == 0) {
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::sendCmd - Response empty");
    lock_.unlock();
    return false;
  }
  if (debug_) {
    stringstream output_hex;
    for (unsigned short j : output) {
      output_hex << hex << j << ' ';
    }
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::sendCmd - Response: %s",
                output_hex.str().c_str());
  }

  if (!checkResponseChecksum(&output) || !checkResponseAck(&output)) {
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::sendCmd - Cmd failed, retry");
    serial_tlt_.flush();
    usleep(300);
    serial_tlt_.write(final_cmd);

    output = feedback();
    if (debug_) {
      stringstream output_hex;
      for (unsigned short j : output) {
        output_hex << hex << j << ' ';
      }
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                  "SerialComTlt::sendCmd - Response: %s",
                  output_hex.str().c_str());
    }
    if (output.size() == 0 || !checkResponseChecksum(&output) ||
        !checkResponseAck(&output)) {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                  "SerialComTlt::sendCmd - Command FAILED !!!");
      lock_.unlock();
      startRs232Com();
      return false;
    }
  }

  if (cmd == "RG") {
    if (*(param->begin()) == 0x11)
      extractPose(&output, 1);
    else if (*(param->begin()) == 0x12)
      extractPose(&output, 2);
  }

  lock_.unlock();
  return true;
}

vector<unsigned char> SerialComTlt::feedback() {
  int i = 0;
  int timeout = 0;
  vector<unsigned char> received_data;
  string last_data;
  string command_type = "";
  int msg_size = -1;
  bool success = false;
  bool first_byte = false;

  while (!stop_loop_) {
    // Check if serial available
    if (serial_tlt_.available()) {

      last_data = serial_tlt_.read();

      if (!first_byte && last_data == "R") { // Beginning of the message
        first_byte = true;
        for (const auto &item : last_data) {
          received_data.push_back(int(item)); // convert cmd string in hex value
        }
      }
      if (first_byte) {
        i++;
        if (i > 1) {
          for (const auto &item : last_data) {
            received_data.push_back(
                int(item)); // convert cmd string in hex value
          }
        }

        // command detector
        if (i == 2) {
          command_type = last_data;
        }

        // success detector
        if (i == 3 && last_data == "") { // ACK
          success = true;
        }

        if (command_type == "G") {
          if (i == 4 && success) {

            for (const auto &item : last_data) {
              msg_size = 7 + int(item);
            }
          } else if (i == 4 && !success) {
            msg_size = 5;
          }
        } else if (command_type == "T" || command_type == "C" ||
                   command_type == "E" || command_type == "S" ||
                   command_type == "O" || command_type == "A") {
          msg_size = 5;
        } else {
          msg_size = 5;
        }

        if (msg_size > 0 && i == msg_size) {
          stop_loop_ = true;
          break;
        }
      }
    } else {
      usleep(1);
      timeout++;
      if (timeout > 5000) {
        stop_loop_ = true;
        return received_data;
      }
    }
  }
  return received_data;
}

/// Compute the checksum
unsigned short SerialComTlt::calculateChecksum(vector<unsigned char> *cmd) {
  unsigned short crc = 0;
  for (vector<unsigned char>::iterator it = cmd->begin(); it != cmd->end();
       ++it) {
    crc = static_cast<unsigned short>(CRC_TABLE[((crc >> 8) ^ *it) & 0xFF] ^
                                      (crc << 8));
  }
  return crc;
}

/// Compare the response checksum with the calculated
bool SerialComTlt::checkResponseChecksum(vector<unsigned char> *response) {
  unsigned short response_checksum_lsb = *(response->end() - 2);
  unsigned short response_checksum_msb = *(response->end() - 1);

  if (debug_) {
    unsigned short checksum =
        (response_checksum_lsb << 8) | response_checksum_msb;
    stringstream checksum_hex;
    checksum_hex << hex << checksum;
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::checkResponseChecksum - Response Checksum = %s",
                checksum_hex.str().c_str());
  }

  vector<unsigned char> response_msg = *response;
  response_msg.resize(response_msg.size() - 2);

  unsigned short checksum = calculateChecksum(&response_msg);
  unsigned short computed_lsb = checksum & 0x00FF;
  unsigned short computed_msb = checksum >> 8;

  if (debug_) {
    unsigned short checksum2 = (computed_lsb << 8) | computed_msb;
    stringstream checksum_hex;
    checksum_hex << hex << checksum2;
    RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                "SerialComTlt::checkResponseChecksum - Computed Checksum = %s",
                checksum_hex.str().c_str());
  }

  if (response_checksum_lsb == computed_lsb &&
      response_checksum_msb == computed_msb) {
    return true;
  } else {
    return false;
  }

  return false;
}

bool SerialComTlt::checkResponseAck(vector<unsigned char> *response) {
  if (response->size() > 4) {
    unsigned short cmd_status = *(response->begin() + 2);
    if (cmd_status == 0x06) {
      return true;
    }

    else if (cmd_status == 0x81) {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                  "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 81 "
                  "Parameter data error ");
    } else if (cmd_status == 0x82) {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                  "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 82 "
                  "Parameter count error");
    } else if (cmd_status == 0x83) {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                  "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 83 "
                  "Command error ");
    } else if (cmd_status == 0x84) {
      RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"),
                  "SerialComTlt::checkResponseAck - Cmd Fail! :  Error 84 "
                  "Permission error ");
    }
  } else {
    return false;
  }
  return false;
}

///  Extract motor pose from the response
bool SerialComTlt::extractPose(vector<unsigned char> *response, int motor) {
  int position = (unsigned char)(*(response->end() - 5)) << 8 |
                 (unsigned char)(*(response->end() - 6));

  if (motor == 1)
    mot1_pose_ = position;
  else if (motor == 2)
    mot2_pose_ = position;
  else
    return false;

  return true;
}

/// Converter
vector<unsigned char> SerialComTlt::intToBytes(int paramInt) {

  vector<unsigned char> arrayOfByte(4);
  for (int i = 0; i < 4; i++)
    arrayOfByte[3 - i] = (paramInt >> (i * 8));
  return arrayOfByte;
}
