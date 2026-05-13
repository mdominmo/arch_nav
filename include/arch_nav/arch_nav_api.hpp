#ifndef ARCH_NAV_ARCH_NAV_API_HPP_
#define ARCH_NAV_ARCH_NAV_API_HPP_

#include <functional>
#include <memory>
#include <vector>

#include "arch_nav/constants/command_response.hpp"
#include "arch_nav/constants/operation_status.hpp"
#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/model/vehicle/global_position.hpp"
#include "arch_nav/model/vehicle/kinematics.hpp"
#include "arch_nav/model/vehicle/vehicle_status.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"
#include "arch_nav/model/vehicle/trajectory_point.hpp"

namespace arch_nav::controller {
class OperationalController;
}
namespace arch_nav::context {
class VehicleContext;
}

namespace arch_nav {

class ArchNavApi {
 public:
  ArchNavApi(controller::OperationalController& controller,
             context::VehicleContext& vehicle_context);
  ~ArchNavApi();

  constants::CommandResponse takeoff(
      double height,
      constants::ReferenceFrame frame = constants::ReferenceFrame::LOCAL_NED);
  constants::CommandResponse land();
  constants::CommandResponse change_yaw(
      double new_yaw,
      constants::ReferenceFrame frame = constants::ReferenceFrame::LOCAL_NED);
  constants::CommandResponse waypoint_following(
      std::vector<vehicle::Waypoint> waypoints,
      constants::ReferenceFrame frame = constants::ReferenceFrame::GLOBAL_WGS84);
  constants::CommandResponse trajectory_execution(
      std::vector<vehicle::TrajectoryPoint> trajectory,
      constants::ReferenceFrame frame = constants::ReferenceFrame::LOCAL_NED);
  void cancel_operation();

  constants::CommandResponse arm();
  constants::CommandResponse disarm();

  constants::OperationStatus       operation_status() const;
  const report::OperationReport*   last_operation_report() const;

  vehicle::GlobalPosition global_position() const;
  vehicle::Kinematics     kinematics() const;
  vehicle::VehicleStatus  vehicle_status() const;

  void on_operation_complete(std::function<void(const report::OperationReport&)> callback);
  void on_operation_progress(std::function<void(const report::OperationReport&)> callback);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace arch_nav

#endif  // ARCH_NAV_ARCH_NAV_API_HPP_
