#ifndef ARCH_NAV_DESCRIPTOR_TAKEOFF_OPERATION_DESCRIPTOR_HPP_
#define ARCH_NAV_DESCRIPTOR_TAKEOFF_OPERATION_DESCRIPTOR_HPP_

#include <memory>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/descriptor/operation_descriptor.hpp"
#include "arch_nav/execution/takeoff_execution_state.hpp"
#include "arch_nav/model/report/takeoff_report.hpp"

namespace arch_nav::descriptor {

class TakeoffOperationDescriptor : public OperationDescriptor {
 public:
  TakeoffOperationDescriptor(double height, constants::ReferenceFrame frame)
      : height_(height),
        frame_(frame),
        progress_(std::make_shared<execution::TakeoffExecutionState>()) {
    progress_->target_altitude.store(height_);
  }

  constants::OperationType operation_type() const override {
    return constants::OperationType::TAKEOFF;
  }

  double height() const { return height_; }
  constants::ReferenceFrame frame() const { return frame_; }

  execution::TakeoffExecutionState& progress() { return *progress_; }
  const execution::TakeoffExecutionState& progress() const {
    return *progress_;
  }

  std::unique_ptr<OperationDescriptor> snapshot() const override {
    auto copy = std::make_unique<TakeoffOperationDescriptor>(height_, frame_);
    copy->progress_->current_altitude.store(
        progress_->current_altitude.load());
    copy->progress_->target_altitude.store(
        progress_->target_altitude.load());
    copy->set_lifecycle_status(lifecycle_status());
    return copy;
  }

  std::shared_ptr<report::OperationReport> make_report() const override {
    return std::make_shared<report::TakeoffReport>(
        progress_, height_, frame_, version());
  }

 private:
  double height_;
  constants::ReferenceFrame frame_;
  std::shared_ptr<execution::TakeoffExecutionState> progress_;
};

}  // namespace arch_nav::descriptor

#endif  // ARCH_NAV_DESCRIPTOR_TAKEOFF_OPERATION_DESCRIPTOR_HPP_
