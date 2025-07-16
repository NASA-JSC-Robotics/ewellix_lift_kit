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

#ifndef LIFTKIT_HARDWARE_INTERFACE__LIFTKIT_HARDWARE_INTERFACE_HPP_
#define LIFTKIT_HARDWARE_INTERFACE__LIFTKIT_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "liftkit_hardware_interface/serial_com_tlt.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "serial/serial.h"
#include "visibility_control.h"

namespace liftkit_hardware_interface
{
class LiftkitHardwareInterface : public hardware_interface::ActuatorInterface
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LiftkitHardwareInterface)

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info_) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  // void signal_callback_handler(int signum);
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_robot_ready_;

  std::vector<double> hw_commands_positions_;

  hardware_interface::HardwareInfo system_info_;
  std::string port_;
  double height_limit_;
  double meters_to_ticks_;
  std::string calibration_direction_;
  double previous_position_;
  SerialComTlt srl_;
  thread com_thread_;

  double min_ticks_mot_1_;  // minimum ticks recordable on motor 1, set in hwi params
  double max_ticks_mot_1_;  // maximum ticks recordable on motor 1, set in hwi params
  double min_ticks_mot_2_;  // minimum ticks recordable on motor 2, set in hwi params
  double max_ticks_mot_2_;  // maximum ticks recordable on motor 2, set in hwi params
  double min_height_m_;     // meters to ticks conversion, set in hwi params
  double max_height_m_;     // meters to ticks conversion, set in hwi params

  bool first_loop_;          // whether or not this is the first run through write
  bool first_non_nan_loop_;  // whether or not this is the first run through write
  bool warned_;              // whether or not the user has been warned

  double dt_;                       // delta time from last loop to this loop
  double last_commanded_position_;  // commanded position last write loop
  rclcpp::Time last_time_;          // time recorded from previous write loop

  double ema_filter_coeff_ = 0.9;  // filter coefficient for EMA, has to be in (0,1)

  // exponential moving average class to help filter velocity
  class EMA
  {
  private:
    double alpha;
    double ema;
    bool first_time;
    bool valid_filter;

  public:
    EMA(double alpha) : alpha(alpha), ema(0), first_time(true), valid_filter(true)
    {
      if (alpha <= 0 || alpha >= 1.0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("LiftkitHardwareInterface"),
                     "Alpha for exponential moving average must be between 0 and 1. The "
                     "filter will just return the most recent value.");
        valid_filter = false;
      }
    }

    void add_value(double value)
    {
      if (!valid_filter)
      {
        ema = value;
      }
      else if (first_time)
      {
        ema = value;
        first_time = false;
      }
      else
      {
        ema = alpha * value + (1 - alpha) * ema;
      }
    }

    double get_average() const
    {
      return ema;
    }
  };

  std::shared_ptr<EMA> desired_vel_ema_;
};
}  // namespace liftkit_hardware_interface

#endif  // LIFTKIT_HARDWARE_INTERFACE__LIFTKIT_HARDWARE_INTERFACE_HPP_
