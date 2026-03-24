#ifndef ARCH_NAV__CORE__CONSTANTS__OPERATION_STATUS_HPP_
#define ARCH_NAV__CORE__CONSTANTS__OPERATION_STATUS_HPP_

#include <cstdint>

namespace arch_nav::constants {

enum class OperationStatus : int32_t {
  HANDOVER = -2,
  DISARMED = -1,
  IDLE     =  0,
  RUNNING  =  1,
  FAILED   =  2
};

}  // namespace arch_nav::constants

#endif  // ARCH_NAV__CORE__CONSTANTS__OPERATION_STATUS_HPP_
