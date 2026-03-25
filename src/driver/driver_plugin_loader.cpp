#include "arch_nav/driver/driver_plugin_loader.hpp"

#include <dirent.h>
#include <dlfcn.h>
#include <sys/stat.h>

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace arch_nav::platform {
namespace {

constexpr const char* kPluginsEnv = "ARCH_NAV_DRIVER_PLUGINS";
constexpr const char* kSearchPathEnv = "ARCH_NAV_DRIVER_PATH";

bool is_regular_file(const std::string& path) {
  struct stat st {};
  return stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

bool starts_with(const std::string& value, const std::string& prefix) {
  return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

bool ends_with(const std::string& value, const std::string& suffix) {
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::vector<std::string> split_env_list(const char* value) {
  std::vector<std::string> result;
  if (value == nullptr || value[0] == '\0') {
    return result;
  }

  std::string current;
  for (const char* it = value;; ++it) {
    if (*it == ':' || *it == '\0') {
      if (current.empty() == false) {
        result.push_back(current);
      }
      current.clear();
      if (*it == '\0') {
        break;
      }
      continue;
    }
    current.push_back(*it);
  }

  return result;
}

std::vector<std::string> collect_search_paths() {
  std::vector<std::string> paths;

  const auto env_paths = split_env_list(std::getenv(kSearchPathEnv));
  paths.insert(paths.end(), env_paths.begin(), env_paths.end());

#ifdef ARCH_NAV_DEFAULT_DRIVER_SEARCH_PATH
  paths.push_back(ARCH_NAV_DEFAULT_DRIVER_SEARCH_PATH);
#endif

  return paths;
}

std::vector<std::string> collect_explicit_plugins() {
  const auto env_plugins = split_env_list(std::getenv(kPluginsEnv));
  return env_plugins;
}

std::vector<std::string> list_plugins_in_directory(const std::string& directory) {
  std::vector<std::string> plugins;

  DIR* dir = opendir(directory.c_str());
  if (dir == nullptr) {
    return plugins;
  }

  constexpr const char* kPrefix = "libarch_nav_";
  constexpr const char* kSuffix = ".so";

  for (dirent* entry = readdir(dir);; entry = readdir(dir)) {
    if (entry == nullptr) {
      break;
    }

    const std::string name(entry->d_name);
    if (name == "." || name == "..") {
      continue;
    }
    if (starts_with(name, kPrefix) == false || ends_with(name, kSuffix) == false) {
      continue;
    }

    const std::string path = directory + "/" + name;
    if (is_regular_file(path)) {
      plugins.push_back(path);
    }
  }

  closedir(dir);
  std::sort(plugins.begin(), plugins.end());
  return plugins;
}

}  // namespace

DriverPluginLoader::~DriverPluginLoader() {
  for (auto it = handles_.rbegin(); it != handles_.rend(); ++it) {
    dlclose(*it);
  }
}

void DriverPluginLoader::load_all() {
  std::unordered_set<std::string> loaded_set(loaded_plugins_.begin(), loaded_plugins_.end());

  for (const auto& path : collect_explicit_plugins()) {
    if (path.empty() || loaded_set.find(path) != loaded_set.end()) {
      continue;
    }
    load_library(path);
    loaded_set.insert(path);
  }

  for (const auto& directory : collect_search_paths()) {
    if (directory.empty()) {
      continue;
    }

    for (const auto& path : list_plugins_in_directory(directory)) {
      if (loaded_set.find(path) != loaded_set.end()) {
        continue;
      }

      void* handle = dlopen(path.c_str(), RTLD_NOW | RTLD_GLOBAL);
      if (handle == nullptr) {
        continue;
      }

      handles_.push_back(handle);
      loaded_plugins_.push_back(path);
      loaded_set.insert(path);
    }
  }
}

const std::vector<std::string>& DriverPluginLoader::loaded_plugins() const {
  return loaded_plugins_;
}

void DriverPluginLoader::load_library(const std::string& path) {
  void* handle = dlopen(path.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (handle == nullptr) {
    const char* error = dlerror();
    throw std::runtime_error(
        "Failed to load driver plugin '" + path + "': " +
        (error == nullptr ? "unknown dlopen error" : error));
  }

  handles_.push_back(handle);
  loaded_plugins_.push_back(path);
}

}  // namespace arch_nav::platform
