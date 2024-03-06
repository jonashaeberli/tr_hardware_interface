#ifndef MERCURY_HARDWARE__MERCURY_HARDWARE_HPP_
#define MERCURY_HARDWARE__MERCURY_HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <odrive_communication/ODrive.h>

using hardware_interface::return_type;

namespace tr_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC MercuryHardware
: public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<double> joint_position_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;

  ODrive::ODrive Hndl;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};

  union
  {
    float f;
    uint32_t u;
  }punning_position;

  union
  {
    float f;
    uint32_t u;
  }punning_velocity;

  std::vector<float> joint_zeros = {9.75, 6.8, -24.75, 0, 1, 0};

};
}  // namespace tr_hardware_interface

#endif  // MERCURY_HARDWARE__MERCURY_HARDWARE_HPP_
