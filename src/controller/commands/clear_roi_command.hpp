#ifndef ARCH_NAV__CORE__CONTROLLER__COMMANDS__CLEAR_ROI_COMMAND_HPP_
#define ARCH_NAV__CORE__CONTROLLER__COMMANDS__CLEAR_ROI_COMMAND_HPP_

#include "arch_nav/context/operation_context.hpp"
#include "controller/vehicle_command.hpp"

namespace arch_nav::controller {

class ClearRoiCommand : public VehicleCommand {
 public:
  explicit ClearRoiCommand(context::OperationContext& operation_context)
      : operation_context_(operation_context) {}

  constants::CommandResponse execute(
      platform::ICommandDispatcher& dispatcher) override {
    auto response = dispatcher.execute_clear_roi();
    if (response == constants::CommandResponse::ACCEPTED) {
      operation_context_.clear_roi();
    }
    return response;
  }

  void cancel(platform::ICommandDispatcher&) override {}

 private:
  context::OperationContext& operation_context_;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__COMMANDS__CLEAR_ROI_COMMAND_HPP_
