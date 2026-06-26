#ifndef ARCH_NAV_MODEL_REPORT_TRAJECTORY_REPORT_HPP_
#define ARCH_NAV_MODEL_REPORT_TRAJECTORY_REPORT_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/execution/trajectory_execution_state.hpp"
#include "arch_nav/model/vehicle/trajectory_point.hpp"

namespace arch_nav::report {

class TrajectoryReport : public OperationReport {
 public:
  TrajectoryReport(
      std::shared_ptr<const execution::TrajectoryExecutionState> state,
      std::vector<vehicle::TrajectoryPoint> trajectory,
      constants::ReferenceFrame frame,
      uint32_t version)
      : state_(std::move(state)),
        trajectory_(std::move(trajectory)),
        frame_(frame),
        version_(version) {}

  const execution::TrajectoryExecutionState& execution_state() const {
    return *state_;
  }

  const std::vector<vehicle::TrajectoryPoint>& trajectory() const {
    return trajectory_;
  }

  constants::ReferenceFrame frame() const { return frame_; }
  uint32_t version() const { return version_; }

 private:
  std::shared_ptr<const execution::TrajectoryExecutionState> state_;
  std::vector<vehicle::TrajectoryPoint> trajectory_;
  constants::ReferenceFrame frame_;
  uint32_t version_;
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV_MODEL_REPORT_TRAJECTORY_REPORT_HPP_
