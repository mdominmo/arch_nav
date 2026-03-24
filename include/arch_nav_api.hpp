#ifndef ARCH_NAV__ARCH_NAV_API_HPP_
#define ARCH_NAV__ARCH_NAV_API_HPP_

#include <functional>
#include <vector>

#include "core/constants/command_response.hpp"
#include "core/constants/operation_status.hpp"
#include "core/constants/reference_frame.hpp"
#include "core/controller/operational_controller.hpp"
#include "core/model/report/operation_report.hpp"
#include "core/model/vehicle/waypoint.hpp"
#include "core/model/vehicle/trajectory_point.hpp"

namespace arch_nav {

class ArchNavApi {
 public:
  explicit ArchNavApi(controller::OperationalController& controller);

  constants::CommandResponse takeoff(
      double height,
      constants::ReferenceFrame frame = constants::ReferenceFrame::LOCAL_NED);
  constants::CommandResponse land();
  constants::CommandResponse waypoint_following(
      std::vector<vehicle::Waypoint> waypoints,
      constants::ReferenceFrame frame);
  constants::CommandResponse trajectory_execution(
      std::vector<vehicle::TrajectoryPoint> trajectory,
      constants::ReferenceFrame frame = constants::ReferenceFrame::LOCAL_NED);
  void cancel_operation();

  constants::CommandResponse arm();
  constants::CommandResponse disarm();

  constants::OperationStatus       operation_status() const;
  const report::OperationReport*   last_operation_report() const;

  void on_operation_complete(std::function<void(const report::OperationReport&)> callback);
  void on_operation_progress(std::function<void(const report::OperationReport&)> callback);

 private:
  controller::OperationalController& controller_;
  std::function<void(const report::OperationReport&)> on_complete_callback_;
  std::function<void(const report::OperationReport&)> on_progress_callback_;
};

}  // namespace arch_nav

#endif  // ARCH_NAV__ARCH_NAV_API_HPP_
