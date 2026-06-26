#ifndef ARCH_NAV_MODEL_REPORT_FOLLOW_TARGET_REPORT_HPP_
#define ARCH_NAV_MODEL_REPORT_FOLLOW_TARGET_REPORT_HPP_

#include <cstdint>
#include <tuple>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/model/report/operation_report.hpp"

namespace arch_nav::report {

class FollowTargetReport : public OperationReport {
 public:
  FollowTargetReport(std::tuple<double, double, double> target_position,
                     double tracking_error,
                     constants::ReferenceFrame frame,
                     uint32_t version)
      : target_position_(target_position),
        tracking_error_(tracking_error),
        frame_(frame),
        version_(version) {}

  std::tuple<double, double, double> target_position() const {
    return target_position_;
  }

  double tracking_error() const { return tracking_error_; }
  constants::ReferenceFrame frame() const { return frame_; }
  uint32_t version() const { return version_; }

 private:
  std::tuple<double, double, double> target_position_;
  double tracking_error_;
  constants::ReferenceFrame frame_;
  uint32_t version_;
};

}  // namespace arch_nav::report

#endif  // ARCH_NAV_MODEL_REPORT_FOLLOW_TARGET_REPORT_HPP_
