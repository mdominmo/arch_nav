#ifndef ARCH_NAV_CONTROLLER_VEHICLE_COMMAND_HPP_
#define ARCH_NAV_CONTROLLER_VEHICLE_COMMAND_HPP_

#include "arch_nav/constants/command_response.hpp"
#include "arch_nav/driver/i_command_dispatcher.hpp"

namespace arch_nav::controller {

class VehicleCommand {
 public:
  virtual constants::CommandResponse execute(platform::ICommandDispatcher& dispatcher) = 0;
  virtual void cancel(platform::ICommandDispatcher& dispatcher) { dispatcher.stop(); }
  virtual ~VehicleCommand() = default;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV_CONTROLLER_VEHICLE_COMMAND_HPP_
