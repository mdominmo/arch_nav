#ifndef ARCH_NAV__CORE__CONSTANTS__REFERENCE_FRAME_HPP_
#define ARCH_NAV__CORE__CONSTANTS__REFERENCE_FRAME_HPP_

namespace arch_nav::constants {

enum class ReferenceFrame {
  GLOBAL_WGS84,
  LOCAL_NED,
  LOCAL_ENU,
  BODY_FCS
};

}  // namespace arch_nav::constants

#endif  // ARCH_NAV__CORE__CONSTANTS__REFERENCE_FRAME_HPP_
