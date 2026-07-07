#include "liftkit_hardware_interface/elmo_liftkit_hardware_interface.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

using namespace std;

namespace liftkit_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn ElmoLiftkitHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Initializing Ewellix Liftkit System");

  if (info_.joints.size() != 1)
  {
    RCLCPP_FATAL(get_logger(), "Expected 1 joint, got %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }

  joint_name_ = info_.joints[0].name;
  state_position_ = 0.0;
  state_velocity_ = 0.0;
  command_position_ = 0.0;

  const auto& p = info_.hardware_parameters;
  auto get_param = [&](const string& key) -> string {
    if (p.find(key) == p.end())
    {
      RCLCPP_FATAL(get_logger(), "Missing parameter: %s", key.c_str());
      throw runtime_error("Missing parameter: " + key);
    }
    return p.at(key);
  };

  try
  {
    port_top_ = get_param("com_port_top");
    port_bottom_ = get_param("com_port_bottom");
    min_height_m_ = stof(get_param("min_height_m"));
    max_height_m_ = stof(get_param("max_height_m"));
    max_ticks_mot_1_ = stoi(get_param("max_ticks_mot_1"));
    max_ticks_mot_2_ = stoi(get_param("max_ticks_mot_2"));
    max_ticks_total_ = max_ticks_mot_1_ + max_ticks_mot_2_;
    homing_current_a_ = stof(get_param("homing_current_a"));
    stall_velocity_thresh_ = stoi(get_param("stall_velocity_thresh"));
    stall_time_ms_ = stoi(get_param("stall_time_ms"));
    homing_timeout_ms_ = stoi(get_param("homing_timeout_ms"));
    poll_ms_ = stoi(get_param("poll_ms"));
    backoff_counts_ = stoi(get_param("backoff_counts"));
    top_home_direction_ = stoi(get_param("top_home_direction"));
    bottom_home_direction_ = stoi(get_param("bottom_home_direction"));
    calibration_direction_ = get_param("calibration_direction");
    height_limit_ = stof(get_param("height_limit"));

    is_fake_hardware_ = (port_top_ == "/dev/null");
  }
  catch (const exception& e)
  {
    RCLCPP_FATAL(get_logger(), "Parameter error: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Liftkit initialized: fake=%d, joint=%s, max_ticks=%d",
              is_fake_hardware_, joint_name_.c_str(), max_ticks_total_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ElmoLiftkitHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  state_position_ = 0.0;
  state_velocity_ = 0.0;
  command_position_ = 0.0;

  if (!is_fake_hardware_)
  {
    elmo_top_ = make_unique<ElmoController>(port_top_, 115200);
    elmo_bottom_ = make_unique<ElmoController>(port_bottom_, 115200);
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ElmoLiftkitHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (elmo_top_) try { elmo_top_->disconnect(); } catch (...) {}
  if (elmo_bottom_) try { elmo_bottom_->disconnect(); } catch (...) {}
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ElmoLiftkitHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_name_, "position", &state_position_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_name_, "velocity", &state_velocity_));

  RCLCPP_INFO(get_logger(), "Exported %zu state interfaces", state_interfaces.size());
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ElmoLiftkitHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(joint_name_, "position", &command_position_));

  RCLCPP_INFO(get_logger(), "Exported %zu command interfaces", command_interfaces.size());
  return command_interfaces;
}

CallbackReturn ElmoLiftkitHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating...");

  try
  {
    if (!is_fake_hardware_)
    {
      elmo_top_->connect();
      elmo_bottom_->connect();
      elmo_top_->setVelocityMode();
      elmo_bottom_->setVelocityMode();
      elmo_top_->motorOn();
      elmo_bottom_->motorOn();
    }

    command_position_ = state_position_;

    RCLCPP_INFO(get_logger(), "Successfully activated!");
  }
  catch (const exception& e)
  {
    RCLCPP_FATAL(get_logger(), "Activation failed: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ElmoLiftkitHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");

  if (elmo_top_ && !is_fake_hardware_)
  {
    try { elmo_top_->motorOff(); } catch (...) {}
  }
  if (elmo_bottom_ && !is_fake_hardware_)
  {
    try { elmo_bottom_->motorOff(); } catch (...) {}
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ElmoLiftkitHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  try
  {
    if (!is_fake_hardware_)
    {
      int32_t top_ticks = elmo_top_->getPosition();
      int32_t bottom_ticks = elmo_bottom_->getPosition();
      int32_t top_vel = elmo_top_->getVelocity();
      int32_t bottom_vel = elmo_bottom_->getVelocity();

      double pos = ((top_ticks + bottom_ticks) / static_cast<double>(max_ticks_total_)) *
                   (max_height_m_ - min_height_m_) + min_height_m_;
      double vel = ((top_vel + bottom_vel) / static_cast<double>(max_ticks_total_)) *
                   (max_height_m_ - min_height_m_);

      state_position_ = pos;
      state_velocity_ = vel;
    }
    else
    {
      // Fake hardware: simulate motion toward target
      double error = command_position_ - state_position_;
      state_position_ = state_position_ + error / 10.0;
      state_velocity_ = error / 10.0;
    }
  }
  catch (const exception& e)
  {
    RCLCPP_WARN(get_logger(), "Read error: %s", e.what());
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ElmoLiftkitHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  try
  {
    if (!is_fake_hardware_)
    {
      double target = min(command_position_, height_limit_);
      double error = target - state_position_;
      int32_t vel_ticks = static_cast<int32_t>(
          error * max_ticks_total_ / (max_height_m_ - min_height_m_));

      elmo_top_->setVelocity(vel_ticks);
      elmo_bottom_->setVelocity(vel_ticks);
    }
    // Fake hardware: write does nothing, read() simulates motion
  }
  catch (const exception& e)
  {
    RCLCPP_WARN(get_logger(), "Write error: %s", e.what());
  }

  return hardware_interface::return_type::OK;
}

}  // namespace liftkit_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    liftkit_hardware_interface::ElmoLiftkitHardwareInterface,
    hardware_interface::SystemInterface)
