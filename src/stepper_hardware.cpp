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
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &position_state_);

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
StepperHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &position_command_);

  return command_interfaces;
}


hardware_interface::return_type
StepperHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  float angle_deg = 0.0f;

  if (stepper_ && stepper_->getPosition(angle_deg))
  {
    position_state_ = angle_deg;
  }
  // else: keep last value, do NOT fail

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type
StepperHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (stepper_)
  {
    float angle_deg = position_command_ * 180.0 / M_PI;
    stepper_->setPosition(static_cast<float>(angle_deg));
  }
  // ignore failure, do NOT kill hardware

  return hardware_interface::return_type::OK;
}


}  // namespace stepper_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  stepper_hardware_interface::StepperHardware,
  hardware_interface::SystemInterface
)
