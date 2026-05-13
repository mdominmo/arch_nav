#ifndef ARCH_NAV__CORE__CONTROLLER__IDLE_STATE_HPP_
#define ARCH_NAV__CORE__CONTROLLER__IDLE_STATE_HPP_

#include "controller/operational_controller.hpp"

namespace arch_nav::controller {

struct OperationalController::IdleState : OperationalController::State {
  void on_vehicle_status_update(
      OperationalController& ctx,
      const vehicle::VehicleStatus& status) override;
  constants::CommandResponse try_execute(
      OperationalController& ctx,
      std::unique_ptr<NavigationTask> task) override;
  constants::CommandResponse try_command(
      OperationalController& ctx,
      std::unique_ptr<VehicleCommand> cmd) override;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__IDLE_STATE_HPP_
