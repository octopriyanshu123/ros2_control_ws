// #include "example_7/arm_hardware.hpp"
// #include <rclcpp/rclcpp.hpp>

// namespace example_7
// {

// hardware_interface::CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
// {
//   if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
//   {
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   // Initialize the joint position and command vectors for 4 joints
//   joint_position_.assign(4, 0.0);
//   joint_position_command_.assign(4, 0.0);

//   RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Robot hardware interface initialized.");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State &)
// {
//   RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Configuring hardware interface.");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State &)
// {
//   RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Activating hardware interface.");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State &)
// {
//   RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Deactivating hardware interface.");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> state_interfaces;
//   for (size_t i = 0; i < joint_position_.size(); ++i)
//   {
//     state_interfaces.emplace_back(hardware_interface::StateInterface(
//       "arm_joint_" + std::to_string(i), hardware_interface::HW_IF_POSITION, &joint_position_[i]));
//   }
//   return state_interfaces;
// }

// std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> command_interfaces;
//   for (size_t i = 0; i < joint_position_command_.size(); ++i)
//   {
//     command_interfaces.emplace_back(hardware_interface::CommandInterface(
//       "arm_joint_" + std::to_string(i), hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
//   }
//   return command_interfaces;
// }

// hardware_interface::return_type RobotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
// {
//   // Simulate reading from hardware (e.g., sensors)
//   for (size_t i = 0; i < joint_position_.size(); ++i)
//   {
//     joint_position_[i] = joint_position_command_[i];  // Simulate real-time position update
//   }

//   RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Reading state from hardware.");
//   return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
// {
//   // Simulate writing to hardware (e.g., actuators)
//   RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Writing commands to hardware.");
//   return hardware_interface::return_type::OK;
// }

// }  // namespace example_7



#include "example_7/arm_hardware.hpp"
#include <rclcpp/rclcpp.hpp>

namespace example_7
{

// RobotSystem::RobotSystem() 
// {
//     // Constructor initializes variables if needed
// }

// RobotSystem::~RobotSystem()
// {
//     // Clean up API instance
// }

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

// Configure the hardware interface
hardware_interface::CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State &)
{
    // Set baud rate and communication port
    int baud_rate_ = 926600;                  // Set baud rate
    std::string com_port_ = "/dev/ttyUSB0";           // Set communication port

    // Create an instance of the AdraApiSerial
    //adra_api_ = new AdraApiSerial(com_port_.c_str(), baud_rate_); // Initialize API with port and baud rate

    RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Configuring hardware interface.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// Activate the hardware interface
hardware_interface::CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Activating hardware interface.");

    // Enable motion for the motors (safety precaution)
    //int ret_into_motion_enable = adra_api_->into_motion_enable();
    int ret_into_motion_enable = 1;
    if (ret_into_motion_enable != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Failed to enable motion.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Set the motion mode to position control (safety precaution)
    //int ret_into_motion_mode_pos = adra_api_->into_motion_mode_pos();
    int ret_into_motion_mode_pos = 1;
    if (ret_into_motion_mode_pos != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Failed to set motion mode to position.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// Deactivate the hardware interface
hardware_interface::CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
    //delete adra_api_; 
    RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Deactivating hardware interface.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// Export state interfaces
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

// Export command interfaces
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

// Read current joint positions from hardware
hardware_interface::return_type RobotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    // Read the current positions of the joints from the hardware
    for (size_t i = 0; i < joint_position_.size(); ++i)
    {
        //int ret = adra_api_->get_pos_current(i, &joint_position_[i]);
        int ret = 1;
        if (ret != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Failed to read current position of joint %zu.", i);
            return hardware_interface::return_type::ERROR;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Reading state from hardware.");
    return hardware_interface::return_type::OK;
}

// Write target positions to hardware
hardware_interface::return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    // Write the target positions to the hardware
    for (size_t i = 0; i < joint_position_command_.size(); ++i)
    {
        //int ret = adra_api_->set_pos_target(i, joint_position_command_[i]);
        int ret = 1;
        if (ret != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Failed to set target position for joint %zu.", i);
            return hardware_interface::return_type::ERROR;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Writing commands to hardware.");
    return hardware_interface::return_type::OK;
}

}  // namespace example_7
