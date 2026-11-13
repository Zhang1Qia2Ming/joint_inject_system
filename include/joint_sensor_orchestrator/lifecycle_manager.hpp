#pragma once

#include "joint_sensor_player/player.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace joint_sensor_orchestrator {

enum class LifecycleState {
  Created,
  Preparing,
  Ready,
  Armed,
  Running,
  Failed,
  Stopping,
  Stopped,
};

struct LifecycleFailure {
  std::string component;
  std::string reason;
};

struct LifecycleSnapshot {
  LifecycleState state{LifecycleState::Created};
  std::vector<LifecycleState> transitions;
  LifecycleFailure first_failure;
  std::vector<LifecycleFailure> additional_failures;
};

class LifecycleManager {
 public:
  LifecycleManager();

  void transition(LifecycleState next);
  void fail(std::string component, std::string reason);
  void request_stop() noexcept;

  LifecycleState state() const;
  LifecycleFailure failure() const;
  std::vector<LifecycleFailure> additional_failures() const;
  LifecycleSnapshot snapshot() const;

  std::shared_ptr<joint_sensor_player::CancellationToken> stop_token() const
      noexcept {
    return stop_token_;
  }

 private:
  mutable std::mutex mutex_;
  LifecycleState state_{LifecycleState::Created};
  std::vector<LifecycleState> transitions_{LifecycleState::Created};
  LifecycleFailure first_failure_;
  std::vector<LifecycleFailure> additional_failures_;
  std::shared_ptr<joint_sensor_player::CancellationToken> stop_token_;
};

}  // namespace joint_sensor_orchestrator
