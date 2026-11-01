#pragma once

#include <foxglove/messages.hpp>
#include <nlohmann/json.hpp>
#include "tasks/auto_aim/aimer.hpp"

namespace tools {
struct EkfModelView {
  foxglove::messages::SceneUpdate scene;
  nlohmann::json selection;
  std::optional<foxglove::messages::FrameTransform> target_tf;
};

// Read-only snapshot: geometry comes directly from EKF, selection from Aimer.
EkfModelView build_ekf_model(
  const auto_aim::Target* target, const auto_aim::AimPoint& aim,
  foxglove::messages::Timestamp timestamp);
}
