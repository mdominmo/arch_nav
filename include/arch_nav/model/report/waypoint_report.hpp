#ifndef ARCH_NAV_MODEL_REPORT_WAYPOINT_REPORT_HPP_
#define ARCH_NAV_MODEL_REPORT_WAYPOINT_REPORT_HPP_

#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/model/report/waypoint_driver_operation_data.hpp"

namespace arch_nav::report {

class WaypointReport : public OperationReport {
 public:
  WaypointDriverOperationData& driver_data() { return driver_data_; }
  const WaypointDriverOperationData& driver_data() const { return driver_data_; }

 private:
  WaypointDriverOperationData driver_data_;
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV_MODEL_REPORT_WAYPOINT_REPORT_HPP_
