#ifndef LIFTKIT_HARDWARE_INTERFACE__ELMO_LIFTKIT_HARDWARE_INTERFACE_HPP_
#define LIFTKIT_HARDWARE_INTERFACE__ELMO_LIFTKIT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "liftkit_hardware_interface/ElmoController.h"
#include "visibility_control.h"

namespace liftkit_hardware_interface
{
class ElmoLiftkitHardwareInterface : public hardware_interface::SystemInterface
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ElmoLiftkitHardwareInterface)

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

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

private:

  // Polling thread for async reads
  std::thread polling_thread_;
  std::atomic<bool> polling_active_{false};
  
  // Cached state from background polling
  std::atomic<int32_t> cached_top_ticks_{0};
  std::atomic<int32_t> cached_bottom_ticks_{0};
  std::atomic<int32_t> cached_top_vel_{0};
  std::atomic<int32_t> cached_bottom_vel_{0};

  // Thread function
  void pollingThreadLoop();
  // Single joint - plain doubles (ros2_control needs double* pointers)
  std::string joint_name_;
  double state_position_;
  double state_velocity_;
  double command_position_;
  double state_position_ticks_ = 0.0; 
  double state_velocity_ticks_per_sec_ = 0.0; 

  // Motor parameters
  std::string port_top_;
  std::string port_bottom_;
  double min_height_m_;
  double max_height_m_;
  int32_t max_ticks_mot_1_;
  int32_t max_ticks_mot_2_;
  int32_t max_ticks_total_;

  float homing_current_a_;
  int32_t stall_velocity_thresh_;
  int stall_time_ms_;
  int homing_timeout_ms_;
  int poll_ms_;
  int32_t backoff_counts_;
  int top_home_direction_;
  int bottom_home_direction_;
  std::string calibration_direction_;
  double height_limit_;

  std::unique_ptr<ElmoController> elmo_top_;
  std::unique_ptr<ElmoController> elmo_bottom_;

  bool is_fake_hardware_;
};
}  // namespace liftkit_hardware_interface

#endif  // LIFTKIT_HARDWARE_INTERFACE__ELMO_LIFTKIT_HARDWARE_INTERFACE_HPP_