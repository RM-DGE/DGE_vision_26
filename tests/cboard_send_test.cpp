#include "io/cboard.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  std::string config_path = argc > 1 ? argv[1] : "configs/cboard.yaml";

  io::CBoard cboard(config_path);

  io::Command command{};
  command.control = true;
  command.shoot = true;       // 固定射击
  command.yaw = 0.2;          // 固定偏航角，单位：rad
  command.pitch = -0.1;        // 固定俯仰角，单位：rad

  std::cout << "Start sending fixed control frame to CBoard..." << std::endl;
  std::cout << "yaw=" << command.yaw << ", pitch=" << command.pitch
            << ", control=" << command.control
            << ", shoot=" << command.shoot << std::endl;

  for (int i = 0; i < 20; ++i) {
    cboard.send(command);
    std::this_thread::sleep_for(100ms);
  }

  return 0;
}
