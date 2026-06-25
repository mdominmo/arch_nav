#ifndef ARCH_NAV_MODEL_REPORT_TRAJECTORY_REPORT_HPP_
#define ARCH_NAV_MODEL_REPORT_TRAJECTORY_REPORT_HPP_

#include <memory>

#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/execution/trajectory_execution_state.hpp"

namespace arch_nav::report {

class TrajectoryReport : public OperationReport {
 public:
  explicit TrajectoryReport(
      std::shared_ptr<const execution::TrajectoryExecutionState> state)
      : state_(std::move(state)) {}

  const execution::TrajectoryExecutionState& execution_state() const {
    return *state_;
  }

 private:
  std::shared_ptr<const execution::TrajectoryExecutionState> state_;
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV_MODEL_REPORT_TRAJECTORY_REPORT_HPP_
