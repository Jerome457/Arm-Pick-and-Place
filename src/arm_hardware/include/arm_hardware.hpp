#pragma once

#include <vector>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace arm_hardware
{

class RobotSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ===============================
  // Joint storage
  // ===============================
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;

  // Map interface name → joint names
  std::map<std::string, std::vector<std::string>> joint_interfaces_;

  // ===============================
  // SERIAL HANDLE (THIS WAS MISSING)
  // ===============================
  int serial_fd_{-1};
};

}  // namespace arm_hardware
