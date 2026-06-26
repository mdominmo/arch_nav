#ifndef ARCH_NAV_CONTROLLER_I_OPERATIONAL_CONTROLLER_HPP_
#define ARCH_NAV_CONTROLLER_I_OPERATIONAL_CONTROLLER_HPP_

#include <functional>
#include <string>
#include <vector>

#include "arch_nav/constants/command_response.hpp"
#include "arch_nav/constants/operation_status.hpp"
#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/controller/preemption_event.hpp"
#include "arch_nav/controller/preemption_info.hpp"
#include "arch_nav/controller/preemption_type.hpp"
#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/model/vehicle/global_position.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"
#include "arch_nav/model/vehicle/trajectory_point.hpp"

namespace arch_nav::controller {

class IOperationalController {
 public:
  virtual ~IOperationalController() = default;

  virtual constants::CommandResponse takeoff(
      double height, constants::ReferenceFrame frame) = 0;
  virtual constants::CommandResponse land() = 0;
  virtual constants::CommandResponse change_yaw(
      double new_yaw, constants::ReferenceFrame frame) = 0;
  virtual constants::CommandResponse waypoint_following(
      std::vector<vehicle::Waypoint> waypoints,
      constants::ReferenceFrame frame) = 0;
  virtual constants::CommandResponse trajectory_execution(
      std::vector<vehicle::TrajectoryPoint> trajectory,
      constants::ReferenceFrame frame) = 0;
  virtual constants::CommandResponse follow_target(
      constants::ReferenceFrame frame) = 0;
  virtual void update_follow_target_position(
      double x, double y, double z) = 0;
  virtual void stop() = 0;

  virtual constants::CommandResponse arm() = 0;
  virtual constants::CommandResponse disarm() = 0;
  virtual constants::CommandResponse set_roi(
      vehicle::GlobalPosition position,
      constants::ReferenceFrame frame) = 0;
  virtual constants::CommandResponse clear_roi() = 0;

  virtual constants::OperationStatus operation_status() const = 0;
  virtual const report::OperationReport* last_operation_report() const = 0;

  virtual void set_on_complete_listener(
      std::function<void(const report::OperationReport&)> callback) = 0;
  virtual void set_on_progress_listener(
      std::function<void(const report::OperationReport&)> callback) = 0;
  virtual void set_on_preemption_event_listener(
      std::function<void(const PreemptionEvent&)> callback) = 0;

  virtual void preempt(PreemptionType type,
                       const PreemptionInfo& info) = 0;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV_CONTROLLER_I_OPERATIONAL_CONTROLLER_HPP_
