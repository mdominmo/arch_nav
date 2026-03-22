#ifndef NAVIGATION__CORE__CONTROLLER__WAYPOINT_TASK_HPP_
#define NAVIGATION__CORE__CONTROLLER__WAYPOINT_TASK_HPP_

#include <functional>
#include <memory>
#include <vector>

#include "core/controller/navigation_task.hpp"
#include "core/model/report/waypoint_report.hpp"
#include "core/model/vehicle/geo_waypoint.hpp"

namespace arch_nav::controller {

class WaypointTask : public NavigationTask {
 public:
  explicit WaypointTask(std::vector<vehicle::GeoWaypoint> waypoints);

  void start(
      context::VehicleContext& context,
      dispatchers::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) override;

  void abort() override;

  std::shared_ptr<report::OperationReport> make_report() override;

 private:
  std::vector<vehicle::GeoWaypoint>          waypoints_;
  std::shared_ptr<report::WaypointReport>    report_;
  dispatchers::ICommandDispatcher*           dispatcher_{nullptr};
};

}  // namespace arch_nav::controller

#endif  // NAVIGATION__CORE__CONTROLLER__WAYPOINT_TASK_HPP_
