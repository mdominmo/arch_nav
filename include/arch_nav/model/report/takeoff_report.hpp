#ifndef ARCH_NAV_MODEL_REPORT_TAKEOFF_REPORT_HPP_
#define ARCH_NAV_MODEL_REPORT_TAKEOFF_REPORT_HPP_

#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/model/report/takeoff_driver_operation_data.hpp"

namespace arch_nav::report {

class TakeoffReport : public OperationReport {
 public:
  TakeoffDriverOperationData& driver_data() { return driver_data_; }
  const TakeoffDriverOperationData& driver_data() const { return driver_data_; }

 private:
  TakeoffDriverOperationData driver_data_;
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV_MODEL_REPORT_TAKEOFF_REPORT_HPP_
