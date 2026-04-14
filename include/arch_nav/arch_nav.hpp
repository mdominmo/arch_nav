#ifndef ARCH_NAV_ARCH_NAV_HPP_
#define ARCH_NAV_ARCH_NAV_HPP_

#include <chrono>
#include <memory>

#include "arch_nav/arch_nav_api.hpp"

namespace arch_nav {

class ArchNav {
 public:
  static std::unique_ptr<ArchNav> create(
      std::chrono::milliseconds context_update_period = std::chrono::milliseconds(20));

  ArchNavApi& api();

  ~ArchNav();

 private:
  struct Impl;
  explicit ArchNav(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace arch_nav

#endif  // ARCH_NAV_ARCH_NAV_HPP_
