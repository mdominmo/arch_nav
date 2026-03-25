#ifndef ARCH_NAV_MODEL_REPORT_WAYPOINT_DRIVER_OPERATION_DATA_HPP_
#define ARCH_NAV_MODEL_REPORT_WAYPOINT_DRIVER_OPERATION_DATA_HPP_

#include <atomic>

#include "arch_nav/model/report/driver_operation_data.hpp"

namespace arch_nav::report {

struct WaypointDriverOperationData : DriverOperationData {
  std::atomic<int> current_waypoint{0};
  std::atomic<int> total_waypoints{0};
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV_MODEL_REPORT_WAYPOINT_DRIVER_OPERATION_DATA_HPP_
