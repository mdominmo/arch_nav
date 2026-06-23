#ifndef ARCH_NAV__CORE__CONTROLLER__COMMANDS__SET_ROI_COMMAND_HPP_
#define ARCH_NAV__CORE__CONTROLLER__COMMANDS__SET_ROI_COMMAND_HPP_

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/context/operation_context.hpp"
#include "arch_nav/model/vehicle/global_position.hpp"
#include "controller/vehicle_command.hpp"

namespace arch_nav::controller {

class SetRoiCommand : public VehicleCommand {
 public:
  explicit SetRoiCommand(
      vehicle::GlobalPosition position,
      constants::ReferenceFrame frame,
      context::OperationContext& operation_context)
      : position_(std::move(position)),
        frame_(frame),
        operation_context_(operation_context) {}

  constants::CommandResponse execute(
      platform::ICommandDispatcher& dispatcher) override {
    auto response = dispatcher.execute_set_roi(position_, frame_);
    if (response == constants::CommandResponse::ACCEPTED) {
      operation_context_.update_roi(position_);
    }
    return response;
  }

  void cancel(platform::ICommandDispatcher&) override {}

 private:
  vehicle::GlobalPosition position_;
  constants::ReferenceFrame frame_;
  context::OperationContext& operation_context_;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__COMMANDS__SET_ROI_COMMAND_HPP_
