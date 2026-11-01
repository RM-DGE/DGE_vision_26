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
    check(model.scene.entities.at(4).cubes.empty() && !model.scene.entities.at(4).lines.empty(),
          "predicted plate must be an outline, not another solid occluder");
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
