#include "liftkit_hardware_interface/elmo_liftkit_hardware_interface.hpp"

#include <cmath>
#include <limits>
#include <algorithm>
#include <thread>

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

    motor_acceleration_ = stoi(get_param("motor_acceleration"));
    motor_deceleration_ = stoi(get_param("motor_deceleration"));
    motor_stop_decel_ = stoi(get_param("motor_stop_decel"));
    motor_speed_profile_ = stoi(get_param("motor_speed_profile"));

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

  // Initialize cached values
  cached_top_ticks_ = 0;
  cached_bottom_ticks_ = 0;
  cached_top_vel_ = 0;
  cached_bottom_vel_ = 0;

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
  polling_active_ = false;
  if (polling_thread_.joinable())
  {
    polling_thread_.join();
  }

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
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_name_, "position_ticks", &state_position_ticks_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_name_, "velocity_ticks_per_sec", &state_velocity_ticks_per_sec_));

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

void ElmoLiftkitHardwareInterface::pollingThreadLoop()
{
  RCLCPP_INFO(get_logger(), "Polling thread started");

  while (polling_active_)
  {
    try
    {
      if (elmo_top_ && elmo_bottom_)
      {
        // Read both motors' position and velocity
        // This runs in background, doesn't block the control loop
        cached_top_ticks_ = elmo_top_->getPosition();
        cached_bottom_ticks_ = elmo_bottom_->getPosition();
        cached_top_vel_ = elmo_top_->getVelocity();
        cached_bottom_vel_ = elmo_bottom_->getVelocity();
      }
    }
    catch (const exception& e)
    {
      RCLCPP_WARN(get_logger(), "Polling thread error: %s", e.what());
    }

    // Small sleep to prevent 100% CPU spin, but keep latency low
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  RCLCPP_INFO(get_logger(), "Polling thread stopped");
}

CallbackReturn ElmoLiftkitHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating...");

  // Sync xacro files before activating
  const char* home = getenv("HOME");
  if (home != nullptr) {
    string script_path = string(home) + "/ewellix_lift_kit/ewellix_liftkit_deploy/scripts/update_xacro_from_yaml.py";
    string sync_command = "python3 " + script_path + " > /dev/null 2>&1";
    int result = system(sync_command.c_str());
    if (result == 0) {
      RCLCPP_INFO(get_logger(), "Xacro files synced");
    } else {
      RCLCPP_WARN(get_logger(), "Failed to sync xacro files");
    }
  }

  try
  {
    if (!is_fake_hardware_)
    {
      elmo_top_->connect();
      elmo_bottom_->connect();

      elmo_top_->wait(500);

      // DISABLE ECHO for faster serial communication
      RCLCPP_INFO(get_logger(), "Disabling echo for faster communication...");
      try {
        elmo_top_->disableEcho();
        elmo_bottom_->disableEcho();
      } catch (const exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to disable echo: %s", e.what());
      }

      // Verify/correct top vs bottom assignment via serial number
      static const map<string, string> kElmoMap = {
          {"20210922", "bottomMotor"},
          {"20210926", "topMotor"}
      };

      string sn_top = elmo_top_->getSerialNumber();
      string sn_bottom = elmo_bottom_->getSerialNumber();

      auto strip_semicolon = [](string& sn) {
      size_t pos = sn.find(';');
      if (pos != string::npos) {
        sn = sn.substr(0, pos);
        }
      };

      strip_semicolon(sn_top);
      strip_semicolon(sn_bottom);

      RCLCPP_INFO(get_logger(), "Top port SN: %s (%s)", sn_top.c_str(),
                  kElmoMap.count(sn_top) ? kElmoMap.at(sn_top).c_str() : "UNKNOWN");
      RCLCPP_INFO(get_logger(), "Bottom port SN: %s (%s)", sn_bottom.c_str(),
                  kElmoMap.count(sn_bottom) ? kElmoMap.at(sn_bottom).c_str() : "UNKNOWN");

      if (kElmoMap.count(sn_top) && kElmoMap.at(sn_top) != "topMotor")
      {
        RCLCPP_WARN(get_logger(), "Port mismatch detected — swapping top/bottom controllers");
        std::swap(elmo_top_, elmo_bottom_);
      }

      elmo_top_->motorOff(); 
      elmo_bottom_->motorOff(); 
      elmo_top_->setPositionMode();
      elmo_bottom_->setPositionMode();
      string ac_cmd = "AC=" + to_string(motor_acceleration_);
      string dc_cmd = "DC=" + to_string(motor_deceleration_);
      string sd_cmd = "SD=" + to_string(motor_stop_decel_);
      string sp_cmd = "SP=" + to_string(motor_speed_profile_);
      
      elmo_top_->sendRawCommand(ac_cmd);
      elmo_bottom_->sendRawCommand(ac_cmd);
      elmo_top_->sendRawCommand(dc_cmd);
      elmo_bottom_->sendRawCommand(dc_cmd);
      elmo_top_->sendRawCommand(sd_cmd);
      elmo_bottom_->sendRawCommand(sd_cmd);
      elmo_top_->sendRawCommand(sp_cmd);
      elmo_bottom_->sendRawCommand(sp_cmd);
      elmo_top_->motorOn();
      elmo_bottom_->motorOn();

      // Read actual current position from hardware before syncing command setpoint
      int32_t top_ticks = elmo_top_->getPosition();
      int32_t bottom_ticks = elmo_bottom_->getPosition();
      int32_t total_ticks = top_ticks + bottom_ticks;

      // CORRECTED FORMULA: scale ticks to height range
      double pos = (static_cast<double>(total_ticks) / max_ticks_total_) * 
                   (max_height_m_ - min_height_m_) + min_height_m_;

      RCLCPP_INFO(get_logger(), "Initial position: Top=%d ticks, Bottom=%d ticks, Total=%d ticks -> %.3f m",
                  top_ticks, bottom_ticks, total_ticks, pos);

      state_position_ = pos;
      command_position_ = state_position_;

      // Initialize cached values with current state
      cached_top_ticks_ = top_ticks;
      cached_bottom_ticks_ = bottom_ticks;
      cached_top_vel_ = 0;
      cached_bottom_vel_ = 0;

      // Start background polling thread
      polling_active_ = true;
      polling_thread_ = std::thread(&ElmoLiftkitHardwareInterface::pollingThreadLoop, this);

      RCLCPP_INFO(get_logger(), "Successfully activated! Polling thread started.");
    }
    else
    {
      // Fake hardware: no polling needed
      state_position_ = 0.0;
      command_position_ = 0.0;
    }
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

  polling_active_ = false;
  if (polling_thread_.joinable())
  {
    polling_thread_.join();
  }

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

// read() now just reads cached atomics — NO serial latency!
hardware_interface::return_type ElmoLiftkitHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  try
  {
    if (!is_fake_hardware_)
    {
      int32_t top_ticks = cached_top_ticks_.load();
      int32_t bottom_ticks = cached_bottom_ticks_.load();
      int32_t top_vel = cached_top_vel_.load();
      int32_t bottom_vel = cached_bottom_vel_.load();

      int32_t total_ticks = top_ticks + bottom_ticks;
      int32_t total_vel = top_vel + bottom_vel;

      // Scaled values (m and m/s)
      double pos = (static_cast<double>(total_ticks) / max_ticks_total_) * 
                   (max_height_m_ - min_height_m_) + min_height_m_;
      double vel = (static_cast<double>(total_vel) / max_ticks_total_) * 
                   (max_height_m_ - min_height_m_);

      // Raw tick values
      state_position_ticks_ = static_cast<double>(total_ticks);
      state_velocity_ticks_per_sec_ = static_cast<double>(total_vel);

      state_position_ = pos;
      state_velocity_ = vel;
    }
    else
    {
      // Fake hardware
      double error = command_position_ - state_position_;
      state_position_ = state_position_ + error / 10.0;
      state_velocity_ = error / 10.0;
      
      // Fake ticks too
      state_position_ticks_ = state_position_ * max_ticks_total_ / (max_height_m_ - min_height_m_);
      state_velocity_ticks_per_sec_ = state_velocity_ * max_ticks_total_ / (max_height_m_ - min_height_m_);
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
      double target = std::max(min_height_m_, std::min(command_position_, height_limit_));
      
      // Convert target height to ticks
      int32_t target_ticks = static_cast<int32_t>(
        (target - min_height_m_) / (max_height_m_ - min_height_m_) * max_ticks_total_
      );
      
      // Split evenly between motors
      int32_t ticks_per_motor = target_ticks / 2;
      
      // Use absolute position mode (direct tick control)
      elmo_top_->setPosition(ticks_per_motor);
      elmo_bottom_->setPosition(ticks_per_motor);
      elmo_top_->beginMotion();
      elmo_bottom_->beginMotion();
    }
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