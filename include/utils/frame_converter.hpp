#ifndef ARCH_NAV__UTILS__FRAME_CONVERTER_HPP_
#define ARCH_NAV__UTILS__FRAME_CONVERTER_HPP_

#include <vector>

#include "core/constants/reference_frame.hpp"
#include "core/model/vehicle/waypoint.hpp"
#include "core/model/vehicle/trajectory_point.hpp"
#include "core/model/vehicle/global_position.hpp"

namespace arch_nav::utils::frame_converter {

vehicle::Waypoint to_ned(
    const vehicle::Waypoint& wp,
    constants::ReferenceFrame frame,
    double heading = 0.0);

std::vector<vehicle::Waypoint> to_ned(
    const std::vector<vehicle::Waypoint>& waypoints,
    constants::ReferenceFrame frame,
    double heading = 0.0);

vehicle::TrajectoryPoint to_ned(
    const vehicle::TrajectoryPoint& pt,
    constants::ReferenceFrame frame,
    double heading = 0.0);

std::vector<vehicle::TrajectoryPoint> to_ned(
    const std::vector<vehicle::TrajectoryPoint>& trajectory,
    constants::ReferenceFrame frame,
    double heading = 0.0);

vehicle::Waypoint global_to_ned(
    const vehicle::Waypoint& wp,
    const vehicle::GlobalPosition& ref);

vehicle::Waypoint ned_to_global(
    const vehicle::Waypoint& wp,
    const vehicle::GlobalPosition& ref);

}  // namespace arch_nav::utils::frame_converter

#endif  // ARCH_NAV__UTILS__FRAME_CONVERTER_HPP_
