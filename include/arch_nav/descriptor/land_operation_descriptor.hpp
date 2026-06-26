#ifndef ARCH_NAV_DESCRIPTOR_LAND_OPERATION_DESCRIPTOR_HPP_
#define ARCH_NAV_DESCRIPTOR_LAND_OPERATION_DESCRIPTOR_HPP_

#include <memory>

#include "arch_nav/descriptor/operation_descriptor.hpp"

namespace arch_nav::descriptor {

class LandOperationDescriptor : public OperationDescriptor {
 public:
  constants::OperationType operation_type() const override {
    return constants::OperationType::LAND;
  }

  std::unique_ptr<OperationDescriptor> snapshot() const override {
    auto copy = std::make_unique<LandOperationDescriptor>();
    copy->set_lifecycle_status(lifecycle_status());
    return copy;
  }

  std::shared_ptr<report::OperationReport> make_report() const override {
    return std::make_shared<report::OperationReport>();
  }
};

}  // namespace arch_nav::descriptor

#endif  // ARCH_NAV_DESCRIPTOR_LAND_OPERATION_DESCRIPTOR_HPP_
