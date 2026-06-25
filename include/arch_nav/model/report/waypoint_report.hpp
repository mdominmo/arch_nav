#ifndef ARCH_NAV_MODEL_REPORT_WAYPOINT_REPORT_HPP_
#define ARCH_NAV_MODEL_REPORT_WAYPOINT_REPORT_HPP_

#include <memory>

#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/execution/waypoint_execution_state.hpp"

namespace arch_nav::report {

class WaypointReport : public OperationReport {
 public:
  explicit WaypointReport(
      std::shared_ptr<const execution::WaypointExecutionState> state)
      : state_(std::move(state)) {}

  const execution::WaypointExecutionState& execution_state() const {
    return *state_;
  }

 private:
  std::shared_ptr<const execution::WaypointExecutionState> state_;
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV_MODEL_REPORT_WAYPOINT_REPORT_HPP_
