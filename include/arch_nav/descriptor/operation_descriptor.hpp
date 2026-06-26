#ifndef ARCH_NAV_DESCRIPTOR_OPERATION_DESCRIPTOR_HPP_
#define ARCH_NAV_DESCRIPTOR_OPERATION_DESCRIPTOR_HPP_

#include <atomic>
#include <cstdint>
#include <memory>

#include "arch_nav/constants/operation_type.hpp"
#include "arch_nav/model/report/operation_report.hpp"

namespace arch_nav::descriptor {

class OperationDescriptor {
 public:
  virtual ~OperationDescriptor() = default;

  virtual constants::OperationType operation_type() const = 0;

  report::ReportStatus lifecycle_status() const {
    return static_cast<report::ReportStatus>(lifecycle_status_.load());
  }

  void set_lifecycle_status(report::ReportStatus status) {
    lifecycle_status_.store(static_cast<int>(status));
  }

  uint32_t version() const { return version_.load(); }
  void increment_version() { version_.fetch_add(1); }

  virtual std::unique_ptr<OperationDescriptor> snapshot() const = 0;
  virtual std::shared_ptr<report::OperationReport> make_report() const = 0;

 private:
  std::atomic<int> lifecycle_status_{
      static_cast<int>(report::ReportStatus::IN_PROGRESS)};
  std::atomic<uint32_t> version_{0};
};

}  // namespace arch_nav::descriptor

#endif  // ARCH_NAV_DESCRIPTOR_OPERATION_DESCRIPTOR_HPP_
