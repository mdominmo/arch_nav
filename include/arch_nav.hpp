#ifndef ARCH_NAV__ARCH_NAV_HPP_
#define ARCH_NAV__ARCH_NAV_HPP_

#include <memory>
#include <string>

#include "navigation_api.hpp"

namespace arch_nav {

class ArchNav {
 public:
  static std::unique_ptr<ArchNav> create(const std::string& config_path);

  NavigationApi& api();

  ~ArchNav();

 private:
  struct Impl;
  explicit ArchNav(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace arch_nav

#endif  // ARCH_NAV__ARCH_NAV_HPP_
