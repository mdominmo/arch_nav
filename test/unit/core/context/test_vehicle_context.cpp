#include <cmath>
#include <limits>
#include <gtest/gtest.h>

#include "arch_nav/context/vehicle_context.hpp"

using arch_nav::constants::ArmState;
using arch_nav::constants::ControlState;
using arch_nav::context::VehicleContext;
using arch_nav::vehicle::Kinematics;
using arch_nav::vehicle::GlobalPosition;
using arch_nav::vehicle::VehicleStatus;

TEST(VehicleContext, DefaultsAreInvalid) {
  VehicleContext manager;
  EXPECT_FALSE(manager.get_global_position().is_valid());
  EXPECT_FALSE(manager.get_kinematic().is_valid());
  EXPECT_FALSE(manager.get_vehicle_status().is_valid());
}

TEST(VehicleContext, UpdateGlobalPositionSkipsNaN) {
  VehicleContext manager;
  GlobalPosition state;
  state.lat = 40.0;
  manager.update(state);
  auto gps = manager.get_global_position();
  EXPECT_DOUBLE_EQ(gps.lat, state.lat);
  EXPECT_TRUE(std::isnan(gps.lon));
  EXPECT_TRUE(std::isnan(gps.alt));
  EXPECT_FALSE(gps.is_valid());
}

TEST(VehicleContext, UpdateKinematicSkipsNaN) {
  VehicleContext manager;
  Kinematics state;
  state.vx = 1.5;
  state.heading = 0.7;
  manager.update(state);
  auto kin = manager.get_kinematic();
  EXPECT_DOUBLE_EQ(kin.vx, 1.5);
  EXPECT_DOUBLE_EQ(kin.heading, 0.7);
  EXPECT_TRUE(std::isnan(kin.x));
}

TEST(VehicleContext, RoiDefaultsToEmpty) {
  VehicleContext manager;
  EXPECT_FALSE(manager.get_roi().has_value());
}

TEST(VehicleContext, UpdateRoiStoresValue) {
  VehicleContext manager;
  GlobalPosition roi{40.0, -3.0, 100.0};
  manager.update_roi(roi);
  auto stored = manager.get_roi();
  ASSERT_TRUE(stored.has_value());
  EXPECT_DOUBLE_EQ(stored->lat, 40.0);
  EXPECT_DOUBLE_EQ(stored->lon, -3.0);
  EXPECT_DOUBLE_EQ(stored->alt, 100.0);
}

TEST(VehicleContext, ClearRoiResetsToEmpty) {
  VehicleContext manager;
  manager.update_roi(GlobalPosition{40.0, -3.0, 100.0});
  manager.clear_roi();
  EXPECT_FALSE(manager.get_roi().has_value());
}

TEST(VehicleContext, UpdateVehicleStatus) {
  VehicleContext manager;
  VehicleStatus state(ControlState::KERNEL_CONTROLLED, ArmState::ARMED);
  manager.update(state);
  auto vs = manager.get_vehicle_status();
  EXPECT_EQ(vs.control_state, ControlState::KERNEL_CONTROLLED);
  EXPECT_EQ(vs.arm_state, ArmState::ARMED);

  VehicleStatus state2(ControlState::UNKNOWN, ArmState::UNKNOWN);
  manager.update(state2);
  vs = manager.get_vehicle_status();
  EXPECT_EQ(vs.control_state, ControlState::UNKNOWN);
  EXPECT_EQ(vs.arm_state, ArmState::UNKNOWN);
}
