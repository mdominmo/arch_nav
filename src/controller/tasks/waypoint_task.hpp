#ifndef ARCH_NAV__CORE__CONTROLLER__WAYPOINT_TASK_HPP_
#define ARCH_NAV__CORE__CONTROLLER__WAYPOINT_TASK_HPP_

#include <functional>
#include <memory>
#include <vector>

#include "controller/navigation_task.hpp"
#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/execution/waypoint_execution_state.hpp"
#include "arch_nav/model/report/waypoint_report.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"

namespace arch_nav::controller {

class WaypointTask : public NavigationTask {
 public:
  WaypointTask(std::vector<vehicle::Waypoint> waypoints,
               constants::ReferenceFrame frame);

  constants::CommandResponse start(
      platform::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) override;

  void abort() override;

  std::shared_ptr<report::OperationReport> make_report() override;
  std::unique_ptr<NavigationTaskMemento> make_memento() const override;

 private:
  std::vector<vehicle::Waypoint>                       waypoints_;
  constants::ReferenceFrame                            frame_;
  std::shared_ptr<execution::WaypointExecutionState>   state_;
  platform::ICommandDispatcher*                        dispatcher_{nullptr};
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__WAYPOINT_TASK_HPP_
