#ifndef ARCH_NAV__CORE__CONTROLLER__DISARMED_STATE_HPP_
#define ARCH_NAV__CORE__CONTROLLER__DISARMED_STATE_HPP_

#include <memory>

#include "controller/operational_controller.hpp"
#include "controller/vehicle_command.hpp"

namespace arch_nav::controller {

struct OperationalController::DisarmedState : OperationalController::State {
  constants::CommandResponse try_command(
      OperationalController& ctx,
      std::unique_ptr<VehicleCommand> cmd) override;
  void on_vehicle_status_update(
      OperationalController& ctx,
      const vehicle::VehicleStatus& status) override;

 private:
  std::unique_ptr<VehicleCommand> cmd_;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__DISARMED_STATE_HPP_
