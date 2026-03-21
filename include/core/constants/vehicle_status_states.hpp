#ifndef NAVIGATION_STATE_REGISTER__CONSTANTS__VEHICLE_STATUS_STATES_HPP_
#define NAVIGATION_STATE_REGISTER__CONSTANTS__VEHICLE_STATUS_STATES_HPP_

#include <cstdint>

namespace arch_nav::constants {

enum class ArmState : int32_t {
  UNKNOWN  = -1,
  DISARMED =  0,
  ARMED    =  1
};

enum class ControlState : int32_t {
  UNKNOWN           = -1,
  KERNEL_CONTROLLED =  0,
  EXTERNAL          =  1
};

}  // namespace arch_nav::constants

#endif  // NAVIGATION_STATE_REGISTER__CONSTANTS__VEHICLE_STATUS_STATES_HPP_
