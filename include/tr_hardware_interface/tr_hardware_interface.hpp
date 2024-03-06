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

namespace mercury_hardware
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

  ODrive::ODrive Hndl;

/// Here we create two maps that store the values of the input command beeing sent to the controller and the values read from the controllers...

  std::unordered_map<std::string, std::vector<std::double>> joint_command_interfaces = {
    {"position", {}}, {"velocity", {}}, {"acceleration", {}}};

  std::unordered_map<std::string, std::vector<std::double>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}, {"acceleration", {}}};

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

  typedef struct {
    uint32_t Position;
    uint32_t Velocity;
  }EncoderEstimates;

};
}  // namespace mercury_hardware

#endif  // MERCURY_HARDWARE__MERCURY_HARDWARE_HPP_
