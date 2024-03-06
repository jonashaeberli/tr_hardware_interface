#include "tr_hardware_interface/tr_hardware_interface.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tr_hardware_interface
{
CallbackReturn MercuryHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
 
  // robot has 6 joints and 2 interfaces
  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MercuryHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MercuryHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  return command_interfaces;
}

return_type MercuryHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    for (int i = 0; i < 6; i++) {
      EncoderEstimates estimates = Hndl.GetEncoderEstimate(i);

      punning_position.u = estimates.Position;
      punning_velocity.u = estimates.Velocity;

      joint_position_[i] = (punning_position.f - joint_zeros[i]) / 15.91549;
      joint_velocities_[i] = (punning_velocity.f / 100) * 6.28318;
    }

  return return_type::OK;
}

return_type MercuryHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (int i = 0; i < 6; i++) {
    punning_position.f = (joint_position_command_[i] * 15.91549) + joint_zeros[i];

    Hndl.SetInputPos(i, punning_position.u, 0, 0);
  }
  return return_type::OK;
}

}  // namespace tr_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tr_hardware_interface::MercuryHardware, hardware_interface::SystemInterface)
