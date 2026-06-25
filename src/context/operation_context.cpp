#include "arch_nav/context/operation_context.hpp"

#include <mutex>

namespace arch_nav::context {

OperationContext::OperationContext() = default;

std::vector<operation::Obstacle> OperationContext::get_obstacles() const {
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  std::vector<operation::Obstacle> result;
  result.reserve(obstacles_.size());
  for (const auto& [_, obstacle] : obstacles_) {
    result.push_back(obstacle);
  }
  return result;
}

void OperationContext::set_obstacle(const operation::Obstacle& obstacle) {
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  obstacles_.insert_or_assign(obstacle.id, obstacle);
  publish_obstacles();
}

void OperationContext::remove_obstacle(const std::string& id) {
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  obstacles_.erase(id);
  publish_obstacles();
}

void OperationContext::clear_obstacles() {
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  obstacles_.clear();
  publish_obstacles();
}

void OperationContext::subscribe_obstacles(
    std::function<void(const std::vector<operation::Obstacle>&)> callback) {
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  obstacles_subscribers_.push_back(callback);
  std::vector<operation::Obstacle> current;
  current.reserve(obstacles_.size());
  for (const auto& [_, obstacle] : obstacles_) {
    current.push_back(obstacle);
  }
  callback(current);
}

void OperationContext::publish_obstacles() {
  std::vector<operation::Obstacle> snapshot;
  snapshot.reserve(obstacles_.size());
  for (const auto& [_, obstacle] : obstacles_) {
    snapshot.push_back(obstacle);
  }
  for (const auto& fn : obstacles_subscribers_) {
    fn(snapshot);
  }
}

}  // namespace arch_nav::context
