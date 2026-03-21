#include <gtest/gtest.h>

#include "core/model/vehicle/vehicle_status.hpp"

using arch_nav::constants::ArmState;
using arch_nav::constants::ControlState;
using arch_nav::vehicle::VehicleStatus;

TEST(VehicleStatus, InitDefaults) {
  VehicleStatus state;
  EXPECT_EQ(state.control_state, ControlState::UNKNOWN);
  EXPECT_EQ(state.arm_state, ArmState::UNKNOWN);
  EXPECT_FALSE(state.is_valid());
}

TEST(VehicleStatus, InitWithStates) {
  VehicleStatus state(ControlState::KERNEL_CONTROLLED, ArmState::ARMED);
  EXPECT_EQ(state.control_state, ControlState::KERNEL_CONTROLLED);
  EXPECT_EQ(state.arm_state, ArmState::ARMED);
  EXPECT_TRUE(state.is_valid());
}

TEST(VehicleStatus, InitWithUnknownArmState) {
  VehicleStatus state(ControlState::KERNEL_CONTROLLED, ArmState::UNKNOWN);
  EXPECT_EQ(state.control_state, ControlState::KERNEL_CONTROLLED);
  EXPECT_EQ(state.arm_state, ArmState::UNKNOWN);
  EXPECT_FALSE(state.is_valid());
}
