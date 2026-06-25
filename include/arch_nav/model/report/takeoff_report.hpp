#ifndef ARCH_NAV_MODEL_REPORT_TAKEOFF_REPORT_HPP_
#define ARCH_NAV_MODEL_REPORT_TAKEOFF_REPORT_HPP_

#include <memory>

#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/execution/takeoff_execution_state.hpp"

namespace arch_nav::report {

class TakeoffReport : public OperationReport {
 public:
  explicit TakeoffReport(
      std::shared_ptr<const execution::TakeoffExecutionState> state)
      : state_(std::move(state)) {}

  const execution::TakeoffExecutionState& execution_state() const {
    return *state_;
  }

 private:
  std::shared_ptr<const execution::TakeoffExecutionState> state_;
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV_MODEL_REPORT_TAKEOFF_REPORT_HPP_
