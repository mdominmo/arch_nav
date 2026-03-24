cmake_minimum_required(VERSION 3.16)
project(arch_nav VERSION 0.0.1 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(arch_nav SHARED
  src/core/controller/operational_controller.cpp
  src/core/controller/states/handover_state.cpp
  src/core/controller/states/disarmed_state.cpp
  src/core/controller/states/idle_state.cpp
  src/core/controller/states/running_state.cpp
  src/core/controller/tasks/land_task.cpp
  src/core/controller/tasks/takeoff_task.cpp
  src/core/controller/tasks/waypoint_task.cpp
  src/core/controller/tasks/trajectory_execution_task.cpp
  src/core/context/vehicle_context.cpp
  src/utils/gnss_local_frame.cpp
  src/utils/frame_converter.cpp
  src/arch_nav_api.cpp
  src/arch_nav_core.cpp
  src/platform/driver_registry.cpp
  src/platform/driver_plugin_loader.cpp
  src/arch_nav.cpp
)

add_library(arch_nav::arch_nav ALIAS arch_nav)

target_compile_features(arch_nav PUBLIC cxx_std_17)

target_link_libraries(arch_nav
  PRIVATE ${CMAKE_DL_LIBS}
)

target_include_directories(arch_nav
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_definitions(arch_nav PRIVATE
  ARCH_NAV_DEFAULT_DRIVER_SEARCH_PATH="${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/arch_nav/drivers"
)

if(BUILD_TESTING)
  include(CTest)
  find_package(GTest REQUIRED)

  function(add_arch_nav_test test_name source_file)
    add_executable(${test_name} ${source_file})
    target_link_libraries(${test_name} PRIVATE arch_nav GTest::gtest_main)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endfunction()

  add_arch_nav_test(test_global_position test/unit/core/state/test_global_position.cpp)
  add_arch_nav_test(test_kinematics test/unit/core/state/test_kinematics.cpp)
  add_arch_nav_test(test_vehicle_status test/unit/core/state/test_vehicle_status.cpp)
  add_arch_nav_test(test_vehicle_context test/unit/core/context/test_vehicle_context.cpp)

  add_arch_nav_test(test_gnss_local_frame test/unit/utils/test_gnss_local_frame.cpp)
endif()

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS arch_nav
  EXPORT arch_navTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

set(ARCH_NAV_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/arch_nav)

install(EXPORT arch_navTargets
  FILE arch_navTargets.cmake
  NAMESPACE arch_nav::
  DESTINATION ${ARCH_NAV_CONFIG_INSTALL_DIR}
)

configure_package_config_file(
  cmake/arch_navConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/arch_navConfig.cmake
  INSTALL_DESTINATION ${ARCH_NAV_CONFIG_INSTALL_DIR}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/arch_navConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/arch_navConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/arch_navConfigVersion.cmake
  DESTINATION ${ARCH_NAV_CONFIG_INSTALL_DIR}
)

# Auto-discover drivers inside drivers/ subdirectory
file(GLOB _driver_cmakes "${CMAKE_CURRENT_SOURCE_DIR}/drivers/*/CMakeLists.txt")
foreach(_driver_cmake ${_driver_cmakes})
  get_filename_component(_driver_dir ${_driver_cmake} DIRECTORY)
  add_subdirectory(${_driver_dir})
endforeach()

# Additional drivers via ARCH_NAV_DRIVERS env var (colon-separated source paths)
if(DEFINED ENV{ARCH_NAV_DRIVERS})
  string(REPLACE ":" ";" _extra_drivers "$ENV{ARCH_NAV_DRIVERS}")
  foreach(_driver_dir ${_extra_drivers})
    if(EXISTS "${_driver_dir}/CMakeLists.txt")
      get_filename_component(_driver_name ${_driver_dir} NAME)
      add_subdirectory(${_driver_dir} ${CMAKE_BINARY_DIR}/drivers/${_driver_name})
    else()
      message(WARNING "ARCH_NAV_DRIVERS: no CMakeLists.txt in ${_driver_dir}, skipping")
    endif()
  endforeach()
endif()
