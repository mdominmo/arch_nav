#ifndef ARCH_NAV_DRIVER_I_COMMAND_DISPATCHER_HPP_
#define ARCH_NAV_DRIVER_I_COMMAND_DISPATCHER_HPP_

#include <functional>
#include <vector>

#include "arch_nav/constants/command_response.hpp"
#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/model/report/takeoff_driver_operation_data.hpp"
#include "arch_nav/model/report/waypoint_driver_operation_data.hpp"
#include "arch_nav/model/vehicle/global_position.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"
#include "arch_nav/model/vehicle/trajectory_point.hpp"

namespace arch_nav::platform {

class ICommandDispatcher {
 public:
  virtual constants::CommandResponse execute_takeoff(
      double, constants::ReferenceFrame,
      std::function<void()>,
      report::TakeoffDriverOperationData&) {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_land(
      std::function<void()>) {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_change_yaw(
      double, constants::ReferenceFrame,
      std::function<void()>) {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_waypoint_following(
      std::vector<vehicle::Waypoint>,
      constants::ReferenceFrame,
      std::function<void()>,
      report::WaypointDriverOperationData&) {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_trajectory(
      std::vector<vehicle::TrajectoryPoint>,
      constants::ReferenceFrame,
      std::function<void()>) {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_arm() {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_disarm() {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_set_roi(
      vehicle::GlobalPosition, constants::ReferenceFrame) {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_clear_roi() {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual void stop() {}

  virtual ~ICommandDispatcher() = default;
};

}  // namespace arch_nav::platform

#endif  // ARCH_NAV_DRIVER_I_COMMAND_DISPATCHER_HPP_
