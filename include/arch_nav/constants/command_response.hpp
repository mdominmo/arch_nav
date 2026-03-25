#ifndef ARCH_NAV_CONSTANTS_COMMAND_RESPONSE_HPP_
#define ARCH_NAV_CONSTANTS_COMMAND_RESPONSE_HPP_

namespace arch_nav::constants {

enum class CommandResponse {
  ACCEPTED,
  NOT_SUPPORTED,
  DENIED
};

}  // namespace arch_nav::constants

#endif  // ARCH_NAV_CONSTANTS_COMMAND_RESPONSE_HPP_
