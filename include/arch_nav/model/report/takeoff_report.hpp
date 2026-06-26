#ifndef ARCH_NAV_MODEL_REPORT_TAKEOFF_REPORT_HPP_
#define ARCH_NAV_MODEL_REPORT_TAKEOFF_REPORT_HPP_

#include <cstdint>
#include <memory>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/execution/takeoff_execution_state.hpp"

namespace arch_nav::report {

class TakeoffReport : public OperationReport {
 public:
  TakeoffReport(
      std::shared_ptr<const execution::TakeoffExecutionState> state,
      double target_height,
      constants::ReferenceFrame frame,
      uint32_t version)
      : state_(std::move(state)),
        target_height_(target_height),
        frame_(frame),
        version_(version) {}

  const execution::TakeoffExecutionState& execution_state() const {
    return *state_;
  }

  double target_height() const { return target_height_; }
  constants::ReferenceFrame frame() const { return frame_; }
  uint32_t version() const { return version_; }

 private:
  std::shared_ptr<const execution::TakeoffExecutionState> state_;
  double target_height_;
  constants::ReferenceFrame frame_;
  uint32_t version_;
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV_MODEL_REPORT_TAKEOFF_REPORT_HPP_
