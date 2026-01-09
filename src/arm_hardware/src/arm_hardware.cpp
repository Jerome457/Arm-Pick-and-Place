#include "arm_hardware.hpp"

#include <string>
#include <vector>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <rclcpp/logging.hpp>

namespace arm_hardware
{

static constexpr size_t REAL_JOINT_INDEX = 2;  // real hardware joint index

hardware_interface::CallbackReturn
RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t num_joints = info_.joints.size();

  joint_position_.assign(num_joints, 0.0);
  joint_velocity_.assign(num_joints, 0.0);
  joint_position_command_.assign(num_joints, 0.0);
  joint_velocity_command_.assign(num_joints, 0.0);

  // Parse joint interfaces
  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces_[interface.name].push_back(joint.name);
    }
  }

  // ---- SERIAL OPEN ----
  serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0)
  {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("arm_hardware"),
      "Serial not available, running all joints as fake");
  }
  else
  {
    struct termios tty{};
    tcgetattr(serial_fd_, &tty);
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag = CS8 | CREAD | CLOCAL;
    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tcsetattr(serial_fd_, TCSANOW, &tty);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  size_t index = 0;
  for (const auto & joint_name :
       joint_interfaces_[hardware_interface::HW_IF_POSITION])
  {
    state_interfaces.emplace_back(
      joint_name,
      hardware_interface::HW_IF_POSITION,
      &joint_position_[index++]);
  }

  index = 0;
  for (const auto & joint_name :
       joint_interfaces_[hardware_interface::HW_IF_VELOCITY])
  {
    state_interfaces.emplace_back(
      joint_name,
      hardware_interface::HW_IF_VELOCITY,
      &joint_velocity_[index++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  size_t index = 0;
  for (const auto & joint_name :
       joint_interfaces_[hardware_interface::HW_IF_POSITION])
  {
    command_interfaces.emplace_back(
      joint_name,
      hardware_interface::HW_IF_POSITION,
      &joint_position_command_[index++]);
  }

  index = 0;
  for (const auto & joint_name :
       joint_interfaces_[hardware_interface::HW_IF_VELOCITY])
  {
    command_interfaces.emplace_back(
      joint_name,
      hardware_interface::HW_IF_VELOCITY,
      &joint_velocity_command_[index++]);
  }

  return command_interfaces;
}

hardware_interface::return_type
RobotSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  // ---------- REAL JOINT ----------
  if (serial_fd_ >= 0)
  {
    char buf[64];
    int bytes = ::read(serial_fd_, buf, sizeof(buf) - 1);
    if (bytes > 0)
    {
      buf[bytes] = '\0';
      double angle_deg = std::stod(buf);
      joint_position_[REAL_JOINT_INDEX] = angle_deg * M_PI / 180.0;
      joint_velocity_[REAL_JOINT_INDEX] = 0.0;
    }
  }
  else
  {
    joint_velocity_[REAL_JOINT_INDEX] =
      joint_velocity_command_[REAL_JOINT_INDEX];
    joint_position_[REAL_JOINT_INDEX] +=
      joint_velocity_command_[REAL_JOINT_INDEX] * period.seconds();
  }

  // ---------- OTHER FAKE JOINTS ----------
  for (size_t i = 0; i < joint_position_.size(); ++i)
  {
    if (i == REAL_JOINT_INDEX)
      continue;

    joint_velocity_[i] = joint_velocity_command_[i];
    joint_position_[i] +=
      joint_velocity_command_[i] * period.seconds();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
RobotSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (serial_fd_ >= 0)
  {
    int target_deg =
      static_cast<int>(
        joint_position_command_[REAL_JOINT_INDEX] * 180.0 / M_PI);
    std::string out = std::to_string(target_deg) + "\n";
    ::write(serial_fd_, out.c_str(), out.size());
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arm_hardware::RobotSystem,
  hardware_interface::SystemInterface)
