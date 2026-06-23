#include <atomic>
#include <gtest/gtest.h>

#include "arch_nav/context/operation_context.hpp"

using arch_nav::context::OperationContext;
using arch_nav::operation::Obstacle;
using arch_nav::vehicle::GlobalPosition;

TEST(OperationContext, RoiDefaultsToEmpty) {
  OperationContext ctx;
  EXPECT_FALSE(ctx.get_roi().has_value());
}

TEST(OperationContext, UpdateRoiStoresValue) {
  OperationContext ctx;
  GlobalPosition roi{40.0, -3.0, 100.0};
  ctx.update_roi(roi);
  auto stored = ctx.get_roi();
  ASSERT_TRUE(stored.has_value());
  EXPECT_DOUBLE_EQ(stored->lat, 40.0);
  EXPECT_DOUBLE_EQ(stored->lon, -3.0);
  EXPECT_DOUBLE_EQ(stored->alt, 100.0);
}

TEST(OperationContext, ClearRoiResetsToEmpty) {
  OperationContext ctx;
  ctx.update_roi(GlobalPosition{40.0, -3.0, 100.0});
  ctx.clear_roi();
  EXPECT_FALSE(ctx.get_roi().has_value());
}

TEST(OperationContext, ObstaclesDefaultToEmpty) {
  OperationContext ctx;
  EXPECT_TRUE(ctx.get_obstacles().empty());
}

TEST(OperationContext, SetObstacleAddsEntry) {
  OperationContext ctx;
  ctx.set_obstacle(Obstacle{"obs1", GlobalPosition{40.0, -3.0, 50.0}, 5.0, 10.0});
  auto obstacles = ctx.get_obstacles();
  ASSERT_EQ(obstacles.size(), 1u);
  EXPECT_EQ(obstacles[0].id, "obs1");
  EXPECT_DOUBLE_EQ(obstacles[0].radius, 5.0);
  EXPECT_DOUBLE_EQ(obstacles[0].height, 10.0);
}

TEST(OperationContext, SetObstacleUpdatesExisting) {
  OperationContext ctx;
  ctx.set_obstacle(Obstacle{"obs1", GlobalPosition{40.0, -3.0, 50.0}, 5.0, 10.0});
  ctx.set_obstacle(Obstacle{"obs1", GlobalPosition{41.0, -2.0, 60.0}, 8.0, 15.0});
  auto obstacles = ctx.get_obstacles();
  ASSERT_EQ(obstacles.size(), 1u);
  EXPECT_DOUBLE_EQ(obstacles[0].radius, 8.0);
  EXPECT_DOUBLE_EQ(obstacles[0].position.lat, 41.0);
}

TEST(OperationContext, RemoveObstacleDeletesEntry) {
  OperationContext ctx;
  ctx.set_obstacle(Obstacle{"obs1", GlobalPosition{40.0, -3.0, 50.0}, 5.0, 10.0});
  ctx.set_obstacle(Obstacle{"obs2", GlobalPosition{41.0, -2.0, 60.0}, 3.0, 8.0});
  ctx.remove_obstacle("obs1");
  auto obstacles = ctx.get_obstacles();
  ASSERT_EQ(obstacles.size(), 1u);
  EXPECT_EQ(obstacles[0].id, "obs2");
}

TEST(OperationContext, ClearObstaclesRemovesAll) {
  OperationContext ctx;
  ctx.set_obstacle(Obstacle{"obs1", GlobalPosition{40.0, -3.0, 50.0}, 5.0, 10.0});
  ctx.set_obstacle(Obstacle{"obs2", GlobalPosition{41.0, -2.0, 60.0}, 3.0, 8.0});
  ctx.clear_obstacles();
  EXPECT_TRUE(ctx.get_obstacles().empty());
}

TEST(OperationContext, SubscribeObstaclesReceivesCurrentState) {
  OperationContext ctx;
  ctx.set_obstacle(Obstacle{"obs1", GlobalPosition{40.0, -3.0, 50.0}, 5.0, 10.0});

  std::vector<Obstacle> received;
  ctx.subscribe_obstacles([&received](const std::vector<Obstacle>& obs) {
    received = obs;
  });

  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].id, "obs1");
}

TEST(OperationContext, SubscribeObstaclesNotifiedOnChange) {
  OperationContext ctx;
  std::atomic<int> call_count{0};
  std::vector<Obstacle> last_received;

  ctx.subscribe_obstacles([&](const std::vector<Obstacle>& obs) {
    last_received = obs;
    call_count++;
  });

  EXPECT_EQ(call_count.load(), 1);

  ctx.set_obstacle(Obstacle{"obs1", GlobalPosition{40.0, -3.0, 50.0}, 5.0, 10.0});
  EXPECT_EQ(call_count.load(), 2);
  ASSERT_EQ(last_received.size(), 1u);

  ctx.set_obstacle(Obstacle{"obs2", GlobalPosition{41.0, -2.0, 60.0}, 3.0, 8.0});
  EXPECT_EQ(call_count.load(), 3);
  ASSERT_EQ(last_received.size(), 2u);

  ctx.remove_obstacle("obs1");
  EXPECT_EQ(call_count.load(), 4);
  ASSERT_EQ(last_received.size(), 1u);

  ctx.clear_obstacles();
  EXPECT_EQ(call_count.load(), 5);
  EXPECT_TRUE(last_received.empty());
}
