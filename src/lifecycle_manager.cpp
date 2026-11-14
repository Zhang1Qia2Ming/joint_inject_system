// 状态只能按允许的边走。第一次 fail 留下原因，后面的失败记进附加列表。
// request_stop 只置位停止开关，不改变当前状态。

#include "joint_sensor_orchestrator/lifecycle_manager.hpp"

#include <stdexcept>
#include <utility>

namespace joint_sensor_orchestrator {
namespace {

bool is_approved_transition(LifecycleState current, LifecycleState next) {
  switch (current) {
    case LifecycleState::Created:
      return next == LifecycleState::Preparing;
    case LifecycleState::Preparing:
      return next == LifecycleState::Ready;
    case LifecycleState::Ready:
      return next == LifecycleState::Armed;
    case LifecycleState::Armed:
      return next == LifecycleState::Running;
    case LifecycleState::Running:
      return next == LifecycleState::Stopping;
    case LifecycleState::Failed:
      return next == LifecycleState::Stopping;
    case LifecycleState::Stopping:
      return next == LifecycleState::Stopped;
    case LifecycleState::Stopped:
      return false;
  }
  return false;
}

}  // namespace

LifecycleManager::LifecycleManager()
    : stop_token_(
          std::make_shared<joint_sensor_player::CancellationToken>()) {}

void LifecycleManager::transition(LifecycleState next) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_approved_transition(state_, next)) {
    throw std::logic_error("非法生命周期状态转换");
  }
  state_ = next;
  transitions_.push_back(next);
}

void LifecycleManager::fail(std::string component, std::string reason) {
  if (component.empty() || reason.empty()) {
    throw std::invalid_argument("故障组件和原因不能为空");
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    LifecycleFailure failure{std::move(component), std::move(reason)};
    if (first_failure_.component.empty()) {
      if (state_ == LifecycleState::Stopped) {
        throw std::logic_error("已停止的生命周期不能进入失败状态");
      }
      first_failure_ = std::move(failure);
      state_ = LifecycleState::Failed;
      transitions_.push_back(LifecycleState::Failed);
    } else {
      additional_failures_.push_back(std::move(failure));
    }
  }
  stop_token_->request_stop();
}

void LifecycleManager::request_stop() noexcept {
  stop_token_->request_stop();
}

LifecycleState LifecycleManager::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

LifecycleFailure LifecycleManager::failure() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return first_failure_;
}

std::vector<LifecycleFailure> LifecycleManager::additional_failures() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return additional_failures_;
}

LifecycleSnapshot LifecycleManager::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {state_, transitions_, first_failure_, additional_failures_};
}

}  // namespace joint_sensor_orchestrator
