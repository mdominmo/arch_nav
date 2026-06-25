#include <gtest/gtest.h>

#include "arch_nav/context/vehicle_context.hpp"
#include "protection_proxy/vehicle_context_writer_proxy.hpp"

using arch_nav::constants::ArmState;
using arch_nav::constants::ControlState;
using arch_nav::context::VehicleContext;
using arch_nav::context::VehicleContextWriterProxy;
using arch_nav::vehicle::GlobalPosition;
using arch_nav::vehicle::Kinematics;
using arch_nav::vehicle::VehicleStatus;

TEST(VehicleContextWriterProxy, UpdateGlobalPosition) {
  VehicleContext ctx;
  VehicleContextWriterProxy proxy(ctx);

  GlobalPosition gp{40.0, -3.0, 100.0};
  proxy.update(gp);

  auto stored = ctx.get_global_position();
  EXPECT_DOUBLE_EQ(stored.lat, 40.0);
  EXPECT_DOUBLE_EQ(stored.lon, -3.0);
  EXPECT_DOUBLE_EQ(stored.alt, 100.0);
}

TEST(VehicleContextWriterProxy, UpdateKinematics) {
  VehicleContext ctx;
  VehicleContextWriterProxy proxy(ctx);

  Kinematics kin;
  kin.vx = 1.5;
  kin.heading = 0.7;
  proxy.update(kin);

  auto stored = ctx.get_kinematic();
  EXPECT_DOUBLE_EQ(stored.vx, 1.5);
  EXPECT_DOUBLE_EQ(stored.heading, 0.7);
}

TEST(VehicleContextWriterProxy, UpdateVehicleStatus) {
  VehicleContext ctx;
  VehicleContextWriterProxy proxy(ctx);

  VehicleStatus status(ControlState::KERNEL_CONTROLLED, ArmState::ARMED);
  proxy.update(status);

  auto stored = ctx.get_vehicle_status();
  EXPECT_EQ(stored.control_state, ControlState::KERNEL_CONTROLLED);
  EXPECT_EQ(stored.arm_state, ArmState::ARMED);
}
