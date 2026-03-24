#ifndef ARCH_NAV__CORE__CONTROLLER__WAYPOINT_TASK_HPP_
#define ARCH_NAV__CORE__CONTROLLER__WAYPOINT_TASK_HPP_

#include <functional>
#include <memory>
#include <vector>

#include "core/controller/navigation_task.hpp"
#include "core/constants/reference_frame.hpp"
#include "core/model/report/waypoint_report.hpp"
#include "core/model/vehicle/waypoint.hpp"

namespace arch_nav::controller {

class WaypointTask : public NavigationTask {
 public:
  WaypointTask(std::vector<vehicle::Waypoint> waypoints,
               constants::ReferenceFrame frame);

  constants::CommandResponse start(
      context::VehicleContext& context,
      dispatchers::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) override;

  void abort() override;

  std::shared_ptr<report::OperationReport> make_report() override;

 private:
  std::vector<vehicle::Waypoint>          waypoints_;
  constants::ReferenceFrame               frame_;
  std::shared_ptr<report::WaypointReport> report_;
  dispatchers::ICommandDispatcher*        dispatcher_{nullptr};
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__WAYPOINT_TASK_HPP_
