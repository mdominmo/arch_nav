#ifndef ARCH_NAV_DESCRIPTOR_FOLLOW_TARGET_OPERATION_DESCRIPTOR_HPP_
#define ARCH_NAV_DESCRIPTOR_FOLLOW_TARGET_OPERATION_DESCRIPTOR_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <tuple>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/descriptor/operation_descriptor.hpp"
#include "arch_nav/model/report/follow_target_report.hpp"

namespace arch_nav::descriptor {

class FollowTargetOperationDescriptor : public OperationDescriptor {
 public:
  explicit FollowTargetOperationDescriptor(constants::ReferenceFrame frame)
      : frame_(frame) {}

  constants::OperationType operation_type() const override {
    return constants::OperationType::FOLLOW_TARGET;
  }

  constants::ReferenceFrame frame() const { return frame_; }

  void update_target_position(double x, double y, double z) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_x_ = x;
    target_y_ = y;
    target_z_ = z;
  }

  std::tuple<double, double, double> target_position() const {
    std::lock_guard<std::mutex> lock(target_mutex_);
    return {target_x_, target_y_, target_z_};
  }

  std::atomic<double> tracking_error{0.0};

  std::unique_ptr<OperationDescriptor> snapshot() const override {
    auto copy = std::make_unique<FollowTargetOperationDescriptor>(frame_);
    auto [x, y, z] = target_position();
    copy->update_target_position(x, y, z);
    copy->tracking_error.store(tracking_error.load());
    copy->set_lifecycle_status(lifecycle_status());
    return copy;
  }

  std::shared_ptr<report::OperationReport> make_report() const override {
    return std::make_shared<report::FollowTargetReport>(
        target_position(), tracking_error.load(), frame_, version());
  }

 private:
  constants::ReferenceFrame frame_;
  mutable std::mutex target_mutex_;
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
};

}  // namespace arch_nav::descriptor

#endif  // ARCH_NAV_DESCRIPTOR_FOLLOW_TARGET_OPERATION_DESCRIPTOR_HPP_
