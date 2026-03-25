#ifndef ARCH_NAV_MODEL_REPORT_TAKEOFF_DRIVER_OPERATION_DATA_HPP_
#define ARCH_NAV_MODEL_REPORT_TAKEOFF_DRIVER_OPERATION_DATA_HPP_

#include <atomic>

#include "arch_nav/model/report/driver_operation_data.hpp"

namespace arch_nav::report {

struct TakeoffDriverOperationData : DriverOperationData {
  std::atomic<double> current_altitude{0.0};
  std::atomic<double> target_altitude{0.0};
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV_MODEL_REPORT_TAKEOFF_DRIVER_OPERATION_DATA_HPP_
