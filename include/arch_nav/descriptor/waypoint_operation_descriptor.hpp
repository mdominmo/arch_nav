#ifndef ARCH_NAV_DESCRIPTOR_WAYPOINT_OPERATION_DESCRIPTOR_HPP_
#define ARCH_NAV_DESCRIPTOR_WAYPOINT_OPERATION_DESCRIPTOR_HPP_

#include <memory>
#include <vector>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/descriptor/operation_descriptor.hpp"
#include "arch_nav/execution/waypoint_execution_state.hpp"
#include "arch_nav/model/report/waypoint_report.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"

namespace arch_nav::descriptor {

class WaypointOperationDescriptor : public OperationDescriptor {
 public:
  WaypointOperationDescriptor(std::vector<vehicle::Waypoint> waypoints,
                               constants::ReferenceFrame frame)
      : waypoints_(std::move(waypoints)),
        frame_(frame),
        progress_(std::make_shared<execution::WaypointExecutionState>()) {
    progress_->total_waypoints.store(
        static_cast<int>(waypoints_.size()));
  }

  constants::OperationType operation_type() const override {
    return constants::OperationType::WAYPOINT_FOLLOWING;
  }

  const std::vector<vehicle::Waypoint>& waypoints() const {
    return waypoints_;
  }

  constants::ReferenceFrame frame() const { return frame_; }

  void set_waypoints(std::vector<vehicle::Waypoint> wps) {
    waypoints_ = std::move(wps);
    progress_->total_waypoints.store(
        static_cast<int>(waypoints_.size()));
    increment_version();
  }

  execution::WaypointExecutionState& progress() { return *progress_; }
  const execution::WaypointExecutionState& progress() const {
    return *progress_;
  }

  std::unique_ptr<OperationDescriptor> snapshot() const override {
    auto copy = std::make_unique<WaypointOperationDescriptor>(
        waypoints_, frame_);
    copy->progress_->current_waypoint.store(
        progress_->current_waypoint.load());
    copy->progress_->total_waypoints.store(
        progress_->total_waypoints.load());
    copy->set_lifecycle_status(lifecycle_status());
    return copy;
  }

  std::shared_ptr<report::OperationReport> make_report() const override {
    return std::make_shared<report::WaypointReport>(
        progress_, waypoints_, frame_, version());
  }

 private:
  std::vector<vehicle::Waypoint> waypoints_;
  constants::ReferenceFrame frame_;
  std::shared_ptr<execution::WaypointExecutionState> progress_;
};

}  // namespace arch_nav::descriptor

#endif  // ARCH_NAV_DESCRIPTOR_WAYPOINT_OPERATION_DESCRIPTOR_HPP_
