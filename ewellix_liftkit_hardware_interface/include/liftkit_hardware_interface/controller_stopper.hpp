#ifndef LIFTKIT_CONTROLLER_STOPPER_HPP_
#define LIFTKIT_CONTROLLER_STOPPER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class ControllerStopper
{
public:
  ControllerStopper() = delete;
  ControllerStopper(const rclcpp::Node::SharedPtr& node, bool stop_controllers_on_startup);
  virtual ~ControllerStopper() = default;

private:
  void robotRunningCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  /*!
   * \brief Queries running stoppable controllers and the controllers are
   * stopped.
   *
   * Queries the controller manager for running controllers and compares the
   * result with the consistent_controllers_. The remaining running controllers
   * are stored in stopped_controllers_ and stopped afterwards.
   */
  void findAndStopControllers();

  /*!
   * \brief Starts the controllers stored in stopped_controllers_.
   *
   */
  void startControllers();

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_manager_srv_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controller_list_srv_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_running_sub_;

  std::vector<std::string> consistent_controllers_;
  std::vector<std::string> stopped_controllers_;

  bool stop_controllers_on_startup_;
  bool robot_running_;
};
#endif  // LIFTKIT_CONTROLLER_STOPPER_HPP_
