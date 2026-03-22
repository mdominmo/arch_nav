#ifndef ARCH_NAV__ARCH_NAV_API_HPP_
#define ARCH_NAV__ARCH_NAV_API_HPP_

#include <vector>

#include "core/constants/operation_status.hpp"
#include "core/controller/operational_controller.hpp"
#include "core/model/report/operation_report.hpp"
#include "core/model/vehicle/geo_waypoint.hpp"

namespace arch_nav {

class ArchNavApi {
 public:
  explicit ArchNavApi(controller::OperationalController& controller);

  void takeoff(double height);
  void land();
  void waypoint_following(std::vector<vehicle::GeoWaypoint> waypoints);
  void cancel_operation();
  constants::OperationStatus        operation_status() const;
  const report::OperationReport*  last_operation_report() const;
  void arm();
  void disarm();

 private:
  controller::OperationalController& controller_;
};

}  // namespace arch_nav

#endif  // ARCH_NAV__ARCH_NAV_API_HPP_
