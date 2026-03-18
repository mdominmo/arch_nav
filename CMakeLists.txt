cmake_minimum_required(VERSION 3.8)
project(arch-nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/core/controller/operational_controller.cpp
  src/core/controller/states/handover_state.cpp
  src/core/controller/states/disarmed_state.cpp
  src/core/controller/states/iddle_state.cpp
  src/core/controller/states/running_state.cpp
  src/core/controller/tasks/land_task.cpp
  src/core/controller/tasks/point_to_point_task.cpp
  src/core/controller/tasks/waypoint_task.cpp
  src/core/context/vehicle_context.cpp
  src/core/planner/kinematic_point_to_point_local_planner.cpp
  src/utils/gnss_local_frame.cpp
  src/config/arch_nav_config_loader.cpp
  src/arch_nav_api.cpp
  src/arch_nav_core.cpp
  src/platform/driver_registry.cpp
  src/arch_nav.cpp
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  geographic_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_global_position
    test/unit/core/state/test_global_position.cpp
  )
  target_include_directories(test_global_position PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_global_position ${PROJECT_NAME})
  set_tests_properties(test_global_position PROPERTIES
    LABELS "unit"
  )

  ament_add_gtest(test_kinematics
    test/unit/core/state/test_kinematics.cpp
  )
  target_include_directories(test_kinematics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_kinematics ${PROJECT_NAME})
  set_tests_properties(test_kinematics PROPERTIES
    LABELS "unit"
  )

  ament_add_gtest(test_vehicle_status
    test/unit/core/state/test_vehicle_status.cpp
  )
  target_include_directories(test_vehicle_status PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_vehicle_status ${PROJECT_NAME})
  set_tests_properties(test_vehicle_status PROPERTIES
    LABELS "unit"
  )

  ament_add_gtest(test_vehicle_context
    test/unit/core/context/test_vehicle_context.cpp
  )
  target_include_directories(test_vehicle_context PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_vehicle_context ${PROJECT_NAME})
  set_tests_properties(test_vehicle_context PROPERTIES
    LABELS "unit"
  )

  ament_add_gtest(test_kinematic_point_to_point_local_planner
    test/test_kinematic_point_to_point_local_planner.cpp
  )
  target_include_directories(test_kinematic_point_to_point_local_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_kinematic_point_to_point_local_planner ${PROJECT_NAME})
  set_tests_properties(test_kinematic_point_to_point_local_planner PROPERTIES
    LABELS "unit"
  )

  ament_add_gtest(test_gnss_local_frame
    test/unit/utils/test_gnss_local_frame.cpp
  )
  target_include_directories(test_gnss_local_frame PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_gnss_local_frame ${PROJECT_NAME})
  set_tests_properties(test_gnss_local_frame PROPERTIES
    LABELS "unit"
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  geographic_msgs
)

ament_package()
