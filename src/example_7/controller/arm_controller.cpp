#include "example_7/arm_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace example_7
{

RobotController::RobotController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RobotController::on_init()
{
  // Declare the joint names and command/state interfaces
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  auto command_interfaces = auto_declare<std::vector<std::string>>("command_interfaces", {});
  auto state_interfaces = auto_declare<std::vector<std::string>>("state_interfaces", {});

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_)
  {
    conf.names.push_back(joint_name + "/position");
  }

  return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_)
  {
    conf.names.push_back(joint_name + "/position");
  }

  return conf;
}

controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
{
  joint_position_command_interface_.clear();
  joint_position_state_interface_.clear();

  // Assign command interfaces
  for (auto & interface : command_interfaces_)
  {
    if (interface.get_interface_name() == "position") {
      joint_position_command_interface_.push_back(interface);
    }
  }

  // Assign state interfaces
  for (auto & interface : state_interfaces_)
  {
    if (interface.get_interface_name() == "position") {
      joint_position_state_interface_.push_back(interface);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Send random values to joint positions between 1 and 5
  for (auto & interface : joint_position_command_interface_) {
    interface.get().set_value(1 + static_cast<double>(rand()) / RAND_MAX * (5 - 1));  // Random value between 1 and 5
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_cleanup(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_error(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace example_7

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(example_7::RobotController, controller_interface::ControllerInterface)
