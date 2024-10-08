#include "example_7/arm_hardware.hpp"
#include <rclcpp/rclcpp.hpp>

namespace example_7
{

hardware_interface::CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize the joint position and command vectors for 4 joints
  joint_position_.assign(4, 0.0);
  joint_position_command_.assign(4, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Robot hardware interface initialized.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Configuring hardware interface.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Activating hardware interface.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Deactivating hardware interface.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_position_.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "arm_joint_" + std::to_string(i), hardware_interface::HW_IF_POSITION, &joint_position_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_position_command_.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "arm_joint_" + std::to_string(i), hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type RobotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Simulate reading from hardware (e.g., sensors)
  for (size_t i = 0; i < joint_position_.size(); ++i)
  {
    joint_position_[i] = joint_position_command_[i];  // Simulate real-time position update
  }

  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Reading state from hardware.");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Simulate writing to hardware (e.g., actuators)
  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Writing commands to hardware.");
  return hardware_interface::return_type::OK;
}

}  // namespace example_7
