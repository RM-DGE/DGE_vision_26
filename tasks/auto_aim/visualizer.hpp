#ifndef AUTO_AIM__VISUALIZER_HPP
#define AUTO_AIM__VISUALIZER_HPP

#include <opencv2/opencv.hpp>

#include <cstddef>
#include <list>
#include <string>

#include "aimer.hpp"
#include "armor.hpp"
#include "io/command.hpp"
#include "solver.hpp"
#include "target.hpp"

namespace auto_aim
{

// 主相机自瞄调试画面：装甲板、跟踪板、瞄准点、云台指令。
// 只负责画图，不参与 Solver/Tracker/Aimer 的状态更新。
void draw_main_view(
  cv::Mat & canvas, const std::list<Armor> & armors, const std::list<Target> & targets,
  Solver & solver, const AimPoint & aim_point, const std::string & state,
  const io::Command & command, std::size_t usb_count);

}  // namespace auto_aim

#endif  // AUTO_AIM__VISUALIZER_HPP
