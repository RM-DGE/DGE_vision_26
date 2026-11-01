#include <cmath>
#include <iostream>
#include <stdexcept>

#include "tasks/auto_aim/aimer.hpp"
#include "tools/Foxglove/ekf_model.hpp"

void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
  try {
    check(argc == 2, "config path required");
    for(double speed : {0., 4., -4., 8., -8.}) {
      auto moving = auto_aim::Target::synthetic(4, speed, .2, .08);
      auto_aim::Aimer test_aimer(argv[1]);
      auto cmd = test_aimer.aim({moving}, {}, 23, false, 0);
      check(cmd.control && test_aimer.debug_aim_point.valid, "visible plate must be selected");
      const auto& selected = test_aimer.debug_aim_point.xyza;
      const double angle = std::remainder(selected[3], 2*M_PI);
      check(std::abs(angle) <= 60*M_PI/180 + 1e-6,
            "aimer followed a rear plate instead of switching to a camera-facing plate");
      if(std::abs(speed)>2) {
        check(speed>0 ? angle < 20*M_PI/180 : angle > -20*M_PI/180,
              "spin policy must prefer incoming plates for both rotation directions");
      }
      const auto view = tools::build_ekf_model(&moving, test_aimer.debug_aim_point, {1,0});
      const auto& lead = view.scene.entities.at(4).lines.at(0).points;
      const auto from = moving.full()[test_aimer.debug_aim_point.armor_id].translation;
      check((Eigen::Vector3d(lead[0].x,lead[0].y,lead[0].z)-from).norm()<1e-9 &&
            (Eigen::Vector3d(lead[1].x,lead[1].y,lead[1].z)-selected.head<3>()).norm()<1e-9,
            "prediction line must connect the actual selected plate to its impact point");
    }
    auto target = auto_aim::Target::synthetic(4, 0, .2, .08);
    auto_aim::Aimer aimer(argv[1]);
    auto command = aimer.aim({target}, {}, 23, false, 0);
    check(command.control && aimer.debug_aim_point.valid, "synthetic target aim invalid");
    const auto aim = aimer.debug_aim_point;
    check(aim.armor_id >= 0 && aim.armor_id < 4, "Aimer must expose actual selected plate");
    check((aim.xyza - target.xyza_list()[aim.armor_id]).norm() < 1e-8,
          "stationary aim must correspond to exposed plate ID");
    auto model = tools::build_ekf_model(&target, aim, {1, 0});
    check(model.target_tf.has_value(), "target-follow transform missing");
    check(std::abs(model.target_tf->translation->x - 4) < 1e-9, "follow center x wrong");
    check(std::abs(model.target_tf->translation->z - .04) < 1e-9, "follow center height wrong");
    check(model.selection["selected_id"] == aim.armor_id, "selection ID mismatch");
    check(model.selection["last_observed_id"] == -1, "synthetic target must not claim observation");
    for (int i = 0; i < 4; ++i) {
      const auto& plate = model.scene.entities.at(i);
      const auto& cube = plate.cubes.at(0);
      const auto truth = target.full()[i];
      check((Eigen::Vector3d(cube.pose->position->x, cube.pose->position->y,
                            cube.pose->position->z) - truth.translation).norm() < 1e-9,
            "rendered plate center differs from EKF");
      check(std::abs(cube.size->y - .135) < 1e-9 && std::abs(cube.size->z - .056) < 1e-9,
            "model must use EKF corner dimensions, not 125mm plate shell");
      const auto& q = *cube.pose->orientation;
      check(Eigen::Quaterniond(q.w, q.x, q.y, q.z).angularDistance(truth.orientation) < 1e-9,
            "rendered orientation must preserve the full EKF quaternion");
      check(model.selection["plates"][i]["id"] == i, "plate ID must remain available in inspector");
      check(model.selection["plates"][i]["selected"] == (i == aim.armor_id), "wrong highlight");
    }
    for (const auto& entity : model.scene.entities)
      check(entity.texts.empty(), "3D geometry must not be obscured by text billboards");
    auto invalid = aim;
    invalid.valid = false;
    auto idle = tools::build_ekf_model(&target, invalid, {2, 0});
    check(idle.selection["selected_id"] == -1 && idle.selection["aim_position"].is_null(),
          "invalid aim must not retain selected plate or future aim");
    check(idle.scene.entities.at(4).lines.empty() && idle.scene.entities.at(4).spheres.empty(),
          "invalid aim ghost must disappear");
    const auto& prediction = model.scene.entities.at(4);
    check(prediction.cubes.empty() && prediction.lines.size() == 1 &&
          prediction.lines[0].type == foxglove::messages::LinePrimitive::LineType::LINE_STRIP &&
          prediction.lines[0].points.size() == 2,
          "prediction must contain only the lead segment, never a ghost armor outline");
    check(prediction.spheres.size() == 1, "predicted aim point must remain visible");
    const auto& predicted_position = *prediction.spheres[0].pose->position;
    check((Eigen::Vector3d(predicted_position.x, predicted_position.y, predicted_position.z) -
           aim.xyza.head<3>()).norm() < 1e-9, "predicted aim point moved");
    auto bad_id = aim;
    bad_id.armor_id = 4;
    check(tools::build_ekf_model(&target, bad_id, {2, 1}).selection["selected_id"] == -1,
          "out of range selection must be rejected");
    auto large = target;
    large.armor_type = auto_aim::big;
    check(std::abs(tools::build_ekf_model(&large, aim, {2, 2}).scene.entities[0].cubes[0].size->y - .230) < 1e-9,
          "large plate must use 230mm width");
    std::vector<uint8_t> encoded(32768);
    size_t encoded_size = 0;
    check(model.scene.encode(encoded.data(), encoded.size(), &encoded_size) == foxglove::FoxgloveError::Ok
          && encoded_size > 0, "scene must serialize with bundled Foxglove SDK");
    auto lost = tools::build_ekf_model(nullptr, aim, {3, 0});
    check(!lost.target_tf && lost.scene.entities.empty() && !lost.scene.deletions.empty(),
          "lost target must clear model");
    check(lost.selection["selected_id"] == -1 && lost.selection["plates"].empty(),
          "lost target must clear diagnostics");
    aimer.aim({}, {}, 23, false, 0);
    check(!aimer.debug_aim_point.valid && aimer.debug_aim_point.armor_id == -1,
          "Aimer must clear selection on target loss");
    std::cout << "Foxglove EKF model tests passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
