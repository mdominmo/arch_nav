#ifndef ARCH_NAV_DESCRIPTOR_TRAJECTORY_OPERATION_DESCRIPTOR_HPP_
#define ARCH_NAV_DESCRIPTOR_TRAJECTORY_OPERATION_DESCRIPTOR_HPP_

#include <memory>
#include <vector>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/descriptor/operation_descriptor.hpp"
#include "arch_nav/execution/trajectory_execution_state.hpp"
#include "arch_nav/model/report/trajectory_report.hpp"
#include "arch_nav/model/vehicle/trajectory_point.hpp"

namespace arch_nav::descriptor {

class TrajectoryOperationDescriptor : public OperationDescriptor {
 public:
  TrajectoryOperationDescriptor(
      std::vector<vehicle::TrajectoryPoint> trajectory,
      constants::ReferenceFrame frame)
      : trajectory_(std::move(trajectory)),
        frame_(frame),
        progress_(std::make_shared<execution::TrajectoryExecutionState>()) {
    progress_->total_points.store(
        static_cast<int>(trajectory_.size()));
  }

  constants::OperationType operation_type() const override {
    return constants::OperationType::TRAJECTORY_EXECUTION;
  }

  const std::vector<vehicle::TrajectoryPoint>& trajectory() const {
    return trajectory_;
  }

  constants::ReferenceFrame frame() const { return frame_; }

  void set_trajectory(std::vector<vehicle::TrajectoryPoint> traj) {
    trajectory_ = std::move(traj);
    progress_->total_points.store(
        static_cast<int>(trajectory_.size()));
    increment_version();
  }

  execution::TrajectoryExecutionState& progress() { return *progress_; }
  const execution::TrajectoryExecutionState& progress() const {
    return *progress_;
  }

  std::unique_ptr<OperationDescriptor> snapshot() const override {
    auto copy = std::make_unique<TrajectoryOperationDescriptor>(
        trajectory_, frame_);
    copy->progress_->current_point_index.store(
        progress_->current_point_index.load());
    copy->progress_->total_points.store(
        progress_->total_points.load());
    copy->set_lifecycle_status(lifecycle_status());
    return copy;
  }

  std::shared_ptr<report::OperationReport> make_report() const override {
    return std::make_shared<report::TrajectoryReport>(
        progress_, trajectory_, frame_, version());
  }

 private:
  std::vector<vehicle::TrajectoryPoint> trajectory_;
  constants::ReferenceFrame frame_;
  std::shared_ptr<execution::TrajectoryExecutionState> progress_;
};

}  // namespace arch_nav::descriptor

#endif  // ARCH_NAV_DESCRIPTOR_TRAJECTORY_OPERATION_DESCRIPTOR_HPP_
