#ifndef ARCH_NAV__DISPATCHERS__I_COMMAND_DISPATCHER_HPP_
#define ARCH_NAV__DISPATCHERS__I_COMMAND_DISPATCHER_HPP_

#include <functional>
#include <vector>

#include "core/constants/command_response.hpp"
#include "core/constants/reference_frame.hpp"
#include "core/model/vehicle/waypoint.hpp"
#include "core/model/vehicle/trajectory_point.hpp"

namespace arch_nav::dispatchers {

class ICommandDispatcher {
 public:
  virtual constants::CommandResponse execute_takeoff(
      double height, constants::ReferenceFrame frame,
      std::function<void()> on_complete) {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_land(
      std::function<void()> on_complete) {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_waypoint_following(
      std::vector<vehicle::Waypoint> waypoints,
      constants::ReferenceFrame frame,
      std::function<void()> on_complete) {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_trajectory(
      std::vector<vehicle::TrajectoryPoint> trajectory,
      constants::ReferenceFrame frame,
      std::function<void()> on_complete) {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_arm() {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual constants::CommandResponse execute_disarm() {
    return constants::CommandResponse::NOT_SUPPORTED;
  }

  virtual void stop() {}

  virtual ~ICommandDispatcher() = default;
};

}  // namespace arch_nav::dispatchers

#endif  // ARCH_NAV__DISPATCHERS__I_COMMAND_DISPATCHER_HPP_
