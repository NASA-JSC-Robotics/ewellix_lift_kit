#ifndef LIFTKIT_HARDWARE_INTERFACE__LIFTKIT_HARDWARE_INTERFACE_HPP_
#define LIFTKIT_HARDWARE_INTERFACE__LIFTKIT_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "liftkit_hardware_interface/serial_com_tlt.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "serial/serial.h"
#include "visibility_control.h"

namespace liftkit_hardware_interface {
class LiftkitHardwareInterface : public hardware_interface::ActuatorInterface {

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LiftkitHardwareInterface)

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info_) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  LIFTKIT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
  // void signal_callback_handler(int signum);
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_robot_ready_;

  std::vector<double> hw_commands_positions_;

  hardware_interface::HardwareInfo system_info;
  std::string port;
  double height_limit;
  double previous_position_;
  SerialComTlt srl_;
  thread com_thread_;
};
} // namespace liftkit_hardware_interface

#endif // LIFTKIT_HARDWARE_INTERFACE__LIFTKIT_HARDWARE_INTERFACE_HPP_
