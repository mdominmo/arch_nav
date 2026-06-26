#ifndef ARCH_NAV_MODEL_REPORT_WAYPOINT_REPORT_HPP_
#define ARCH_NAV_MODEL_REPORT_WAYPOINT_REPORT_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/execution/waypoint_execution_state.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"

namespace arch_nav::report {

class WaypointReport : public OperationReport {
 public:
  WaypointReport(
      std::shared_ptr<const execution::WaypointExecutionState> state,
      std::vector<vehicle::Waypoint> waypoints,
      constants::ReferenceFrame frame,
      uint32_t version)
      : state_(std::move(state)),
        waypoints_(std::move(waypoints)),
        frame_(frame),
        version_(version) {}

  const execution::WaypointExecutionState& execution_state() const {
    return *state_;
  }

  const std::vector<vehicle::Waypoint>& waypoints() const {
    return waypoints_;
  }

  constants::ReferenceFrame frame() const { return frame_; }
  uint32_t version() const { return version_; }

 private:
  std::shared_ptr<const execution::WaypointExecutionState> state_;
  std::vector<vehicle::Waypoint> waypoints_;
  constants::ReferenceFrame frame_;
  uint32_t version_;
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV_MODEL_REPORT_WAYPOINT_REPORT_HPP_
