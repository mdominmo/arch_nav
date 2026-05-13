#ifndef ARCH_NAV__CORE__CONTROLLER__COMMANDS__CLEAR_ROI_COMMAND_HPP_
#define ARCH_NAV__CORE__CONTROLLER__COMMANDS__CLEAR_ROI_COMMAND_HPP_

#include "controller/vehicle_command.hpp"

namespace arch_nav::controller {

class ClearRoiCommand : public VehicleCommand {
 public:
  constants::CommandResponse execute(
      platform::ICommandDispatcher& dispatcher) override {
    return dispatcher.execute_clear_roi();
  }

  void cancel(platform::ICommandDispatcher&) override {}
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__COMMANDS__CLEAR_ROI_COMMAND_HPP_
