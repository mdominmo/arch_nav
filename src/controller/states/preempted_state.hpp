#ifndef ARCH_NAV__CORE__CONTROLLER__PREEMPTED_STATE_HPP_
#define ARCH_NAV__CORE__CONTROLLER__PREEMPTED_STATE_HPP_

#include <memory>

#include "controller/operational_controller.hpp"
#include "controller/navigation_task.hpp"
#include "controller/navigation_task_memento.hpp"
#include "arch_nav/controller/preemption_info.hpp"
#include "arch_nav/controller/preemption_type.hpp"

namespace arch_nav::controller {

struct OperationalController::PreemptedState : OperationalController::State {
  PreemptedState(std::unique_ptr<NavigationTaskMemento> memento,
                 std::shared_ptr<report::OperationReport> user_report,
                 PreemptionType preemption_type,
                 PreemptionInfo preemption_info);

  void on_vehicle_status_update(
      OperationalController& ctx,
      const vehicle::VehicleStatus& status) override;

  constants::CommandResponse try_execute(
      OperationalController& ctx,
      std::unique_ptr<NavigationTask> task) override;

  void try_stop(OperationalController& ctx) override;
  PreemptionResult try_preempt(OperationalController& ctx) override;

  void on_supervisor_task_complete(OperationalController& ctx);

 private:
  std::unique_ptr<NavigationTaskMemento> memento_;
  std::shared_ptr<report::OperationReport> user_report_;
  PreemptionType preemption_type_;
  PreemptionInfo preemption_info_;
  std::unique_ptr<NavigationTask> supervisor_task_;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__PREEMPTED_STATE_HPP_
