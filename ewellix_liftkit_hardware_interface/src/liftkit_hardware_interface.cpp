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

#include "liftkit_hardware_interface/liftkit_hardware_interface.hpp"

#include <math.h>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std;

namespace liftkit_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn LiftkitHardwareInterface::on_init(const hardware_interface::HardwareInfo& info_)
{
  if (hardware_interface::ActuatorInterface::on_init(info_) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
  hw_states_robot_ready_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());

  hw_commands_positions_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
  // signal(SIGINT, signal_callback_handler);
  system_info = info_;
  port = system_info.hardware_parameters["com_port"];
  height_limit = stof(system_info.hardware_parameters["height_limit"]);

  first_loop = true;
  first_non_nan_loop = true;

  desired_vel_ema_ = std::make_shared<EMA>(ema_filter_coeff);
  return CallbackReturn::SUCCESS;
}

CallbackReturn LiftkitHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  srl_.height_limit_ = height_limit;
  RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "Successfully configured!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LiftkitHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  srl_.stopRs232Com();  // Com stopped
  srl_.run_ = false;    // stop RC thread loop
  com_thread_.join();
  RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "Successfully cleanup!");
  return CallbackReturn::SUCCESS;
}

vector<hardware_interface::StateInterface> LiftkitHardwareInterface::export_state_interfaces()
{
  vector<hardware_interface::StateInterface> state_interfaces;

  // export sensor state interface
  for (uint i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, info_.joints[i].state_interfaces[1].name, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, info_.joints[i].state_interfaces[2].name, &hw_states_robot_ready_[i]));
  }

  return state_interfaces;
}

vector<hardware_interface::CommandInterface> LiftkitHardwareInterface::export_command_interfaces()
{
  vector<hardware_interface::CommandInterface> command_interfaces;

  // export command state interface
  for (uint i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_commands_positions_[i]));
  }

  return command_interfaces;
}

CallbackReturn LiftkitHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  for (unsigned int i = 0; i < hw_states_positions_.size(); ++i)
  {
    hw_states_positions_[i] = 0;
    hw_states_velocities_[i] = 0;
    hw_states_robot_ready_[i] = 0;
  }

  // // Trying to instantiate the driver
  try
  {
    if (srl_.startSerialCom(port, 38400))
    {
      com_thread_ = thread(&SerialComTlt::comLoop, &srl_);  //  RC thread
      if (srl_.startRs232Com())
      {  // Com started
        RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "Successfully connected!");
      }
      else
      {
        RCLCPP_FATAL(rclcpp::get_logger("LiftkitHardwareInterface"), "Is the Liftkit USB Connected?");
        return CallbackReturn::ERROR;
      }
    }
  }
  catch (boost::system::system_error& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("LiftkitHardwareInterface"), "TCP error: '%s'", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("LiftkitHardwareInterface"), "Successfully activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LiftkitHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  srl_.stopRs232Com();  // Com stopped
  srl_.run_ = false;    // stop RC thread loop
  com_thread_.join();
  RCLCPP_INFO(rclcpp::get_logger("LiftkitHardwareInterface"), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type LiftkitHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // Address compiler warnings.
  (void)time;
  (void)period;

  previous_position_ = hw_states_positions_[0];
  hw_states_velocities_[0] = srl_.current_velocity_;
  hw_states_robot_ready_[0] = 1.0;

  // There is an issue when homing the robot that causes the position to read as
  // an epsilon less than 0 (e.g. -0.001), violating joint limits. If that is
  // the case we simply round the pose to 0 from the HW interface. Otherwise,
  // something may be more seriously wrong so we report the value from the
  // serial comms.
  auto pos = srl_.current_pose_;
  if (pos < 0 && pos > -0.01)
  {
    pos = 0.0;
  }
  hw_states_positions_[0] = pos;

  RCLCPP_DEBUG(rclcpp::get_logger("LiftkitHardwareInterface"), "Reading positions: %f", hw_states_positions_[0]);
  RCLCPP_DEBUG(rclcpp::get_logger("LiftkitHardwareInterface"), "Reading velocities: %f", hw_states_velocities_[0]);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LiftkitHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // Address compiler warnings.
  (void)time;
  (void)period;

  // if it is the first loop, there is no way to calculate dt, so use
  // the expected period instead. Otherwise, calculate dt each loop
  if (first_loop)
  {
    dt = period.seconds();
    first_loop = false;
  }
  else
  {
    dt = (time - last_time).seconds();
  }

  // if we have a nan value, don't command a new position, and set velocity to
  // zero
  if (isnan(hw_commands_positions_[0]))
  {
    desired_vel_ema_->add_value(0);
    srl_.desired_velocity_ = desired_vel_ema_->get_average();
  }
  else
  {
    if (first_non_nan_loop)
    {
      last_commanded_position = hw_commands_positions_[0];
      first_non_nan_loop = false;
    }
    warned_ = false;
    if (hw_commands_positions_[0] > height_limit)
    {
      srl_.desired_pose_ = height_limit;
      if (!warned_)
      {
        RCLCPP_WARN(rclcpp::get_logger("LiftkitHardwareInterface"),
                    "Commanded Height of %0.3f was greater than height limit of "
                    "%0.3f! height "
                    "being clamped.",
                    hw_commands_positions_[0], height_limit);
        warned_ = true;
      }
    }
    else if (hw_commands_positions_[0] <= height_limit && warned_)
    {
      warned_ = false;
      srl_.desired_pose_ = hw_commands_positions_[0];
    }
    else
    {
      srl_.desired_pose_ = hw_commands_positions_[0];
    }

    // filter the desired velocity and set it in the ewellix class
    double raw_velocity = (srl_.desired_pose_ - last_commanded_position) / dt;
    desired_vel_ema_->add_value(raw_velocity);
    srl_.desired_velocity_ = desired_vel_ema_->get_average();
  }

  // store current values to reference next loop
  last_time = time;
  last_commanded_position = srl_.desired_pose_;

  return hardware_interface::return_type::OK;
}
}  // namespace liftkit_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(liftkit_hardware_interface::LiftkitHardwareInterface, hardware_interface::ActuatorInterface)
