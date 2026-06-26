#ifndef ARCH_NAV_DESCRIPTOR_CHANGE_YAW_OPERATION_DESCRIPTOR_HPP_
#define ARCH_NAV_DESCRIPTOR_CHANGE_YAW_OPERATION_DESCRIPTOR_HPP_

#include <memory>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/descriptor/operation_descriptor.hpp"

namespace arch_nav::descriptor {

class ChangeYawOperationDescriptor : public OperationDescriptor {
 public:
  ChangeYawOperationDescriptor(double target_yaw,
                                constants::ReferenceFrame frame)
      : target_yaw_(target_yaw), frame_(frame) {}

  constants::OperationType operation_type() const override {
    return constants::OperationType::CHANGE_YAW;
  }

  double target_yaw() const { return target_yaw_; }
  constants::ReferenceFrame frame() const { return frame_; }

  std::unique_ptr<OperationDescriptor> snapshot() const override {
    auto copy = std::make_unique<ChangeYawOperationDescriptor>(
        target_yaw_, frame_);
    copy->set_lifecycle_status(lifecycle_status());
    return copy;
  }

  std::shared_ptr<report::OperationReport> make_report() const override {
    return std::make_shared<report::OperationReport>();
  }

 private:
  double target_yaw_;
  constants::ReferenceFrame frame_;
};

}  // namespace arch_nav::descriptor

#endif  // ARCH_NAV_DESCRIPTOR_CHANGE_YAW_OPERATION_DESCRIPTOR_HPP_
