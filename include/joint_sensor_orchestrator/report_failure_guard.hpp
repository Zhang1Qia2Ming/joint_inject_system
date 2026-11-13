#pragma once

#include "joint_sensor_orchestrator/report_v2.hpp"

#include <string>
#include <utility>

namespace joint_sensor_orchestrator {

class ReportFailureGuard {
 public:
  ReportFailureGuard(ReportV2& report, ReportV2Summary base)
      : report_(report), summary_(std::move(base)) {}
  ReportFailureGuard(const ReportFailureGuard&) = delete;
  ReportFailureGuard& operator=(const ReportFailureGuard&) = delete;

  ~ReportFailureGuard() noexcept {
    if (!active_) return;
    try {
      if (summary_.first_failure.component.empty()) {
        set_failure("main", "报告作用域异常退出");
      }
      report_.finish(summary_);
    } catch (...) {
    }
  }

  void mark_execution_attempted() noexcept { execution_attempted_ = true; }

  void set_failure(std::string component, std::string reason) {
    summary_.ok = false;
    summary_.final_state = LifecycleState::Failed;
    summary_.stop_reason = reason;
    summary_.first_failure = {std::move(component), std::move(reason)};
    summary_.validation.software_logic =
        execution_attempted_ ? ValidationStatus::Fail
                             : ValidationStatus::NotRun;
  }

  void complete(const ReportV2Summary& summary) {
    report_.finish(summary);
    active_ = false;
  }

  void dismiss() noexcept { active_ = false; }

 private:
  ReportV2& report_;
  ReportV2Summary summary_;
  bool execution_attempted_{};
  bool active_{true};
};

}  // namespace joint_sensor_orchestrator
