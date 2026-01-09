#include "stepper_hardware_interface/stepper_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace stepper_hardware_interface
{

hardware_interface::CallbackReturn
StepperHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  port_ = info.hardware_parameters.at("port");

  serial_ = std::make_unique<motion_sdk::USBSerial>(port_, 115200);

  if (!serial_->open())
  {
    RCLCPP_ERROR(logger_, "Failed to open serial port");
    return CallbackReturn::ERROR;
  }

  stepper_ = std::make_unique<motion_sdk::Stepper>(*serial_);

  if (!stepper_->ping())
  {
    RCLCPP_ERROR(logger_, "Stepper ping failed");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Stepper hardware initialized");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
StepperHardware::export_state_interfaces()
{
  return {
    {info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_state_}
  };
}

std::vector<hardware_interface::CommandInterface>
StepperHardware::export_command_interfaces()
{
  return {
    {info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_command_}
  };
}

hardware_interface::return_type
StepperHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  float angle_deg = 0.0f;

  if (!stepper_->getPosition(angle_deg))
    return hardware_interface::return_type::ERROR;

  position_state_ = angle_deg;  // radians/deg mapping optional
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
StepperHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!stepper_->setPosition(static_cast<float>(position_command_)))
    return hardware_interface::return_type::ERROR;

  return hardware_interface::return_type::OK;
}

}  // namespace stepper_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  stepper_hardware_interface::StepperHardware,
  hardware_interface::SystemInterface
)
