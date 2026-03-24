#ifndef ARCH_NAV__CORE__MODEL__VEHICLE__VEHICLE_STATUS_HPP_
#define ARCH_NAV__CORE__MODEL__VEHICLE__VEHICLE_STATUS_HPP_

#include "core/constants/vehicle_status_states.hpp"

namespace arch_nav::vehicle {

using constants::ArmState;
using constants::ControlState;

struct VehicleStatus {
  ControlState control_state;
  ArmState     arm_state;

  explicit VehicleStatus(
      ControlState control_state_value = ControlState::UNKNOWN,
      ArmState arm_state_value = ArmState::UNKNOWN)
      : control_state(control_state_value), arm_state(arm_state_value) {}

  bool is_valid() const {
    return control_state != ControlState::UNKNOWN &&
           arm_state != ArmState::UNKNOWN;
  }
};

}  // namespace arch_nav::vehicle

#endif  // ARCH_NAV__CORE__MODEL__VEHICLE__VEHICLE_STATUS_HPP_
