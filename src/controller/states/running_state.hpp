#ifndef ARCH_NAV__CORE__CONTROLLER__RUNNING_STATE_HPP_
#define ARCH_NAV__CORE__CONTROLLER__RUNNING_STATE_HPP_

#include <memory>

#include "controller/operational_controller.hpp"
#include "controller/navigation_task.hpp"

namespace arch_nav::controller {

struct OperationalController::RunningState : OperationalController::State {
  explicit RunningState(std::unique_ptr<NavigationTask> task);

  void on_vehicle_status_update(
      OperationalController& ctx,
      const vehicle::VehicleStatus& status) override;
  void try_stop(OperationalController& ctx) override;

 private:
  std::unique_ptr<NavigationTask> task_;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__RUNNING_STATE_HPP_
