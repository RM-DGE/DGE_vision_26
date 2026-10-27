# DGE 文档索引

## 当前状态

- EKF 数学基线：v0.33。
- v0.40：ROS2 桥接、IMU/相机时间戳和延时测试；**当前调试分支已把 ROS 接入从 `src/` 移除**。
- `sentry_DGE` 现在跑纯自瞄路径，不初始化 DDS；通信测试用 `mcu_ros_delay_test`。
- `src/sentry_DGE.cpp` 只做参数解析、模块构造和主循环；全向调试画面在
  `tasks/omniperception/visualizer.cpp`。

| 文档 | 内容 |
|---|---|
| [标定程序.md](./标定程序.md) | 教程：机械零点/杆臂/rpy、QD 风格 URDF 外参与自动 pitch、刚性检查和世界系验收 |
| [EKF调试.md](./EKF调试.md) | 配合 Foxglove 的 EKF 实操调试：看 3D/曲线、查外参、NIS/σ 判读、机动与调参流程 |
| [融合说明.md](./融合说明.md) | 同济 sp_vision × 南京理工 rmcs 的融合内容、改动文件、使用方式 |
| [RobotModel4_v0.3.md](./RobotModel4_v0.3.md) | 4 板像素 EKF v0.3：P0/Q×dt/门控/重初始化/NIS 口径 |
| [RobotModel4_v0.32.md](./RobotModel4_v0.32.md) | v0.32 修改记录：RobustPnp 未定义行为修复、Solver 位姿初始化、仿真 + Foxglove 测试清单 |
| [RobotModel4_v0.33.md](./RobotModel4_v0.33.md) | v0.33 修改记录：时间更新/状态外推拆分、observable 懒刷新、Aimer/Planner/Foxglove 迁移、CWNA/DWNA/DIAG_RATE 过程噪声 |
| [RobotModel4_v0.40.md](./RobotModel4_v0.40.md) | v0.40 修改记录：ROS2 桥接、IMU/状态上行、导航目标下行、时间戳口径与延时测试 |

> v0.33 仍是 EKF 数学基线；v0.40 是通信与时间戳补充，不改变 EKF 的预测/观测逻辑。

## 快速入口

```bash
# 标定
./build/capture configs/calibration.yaml -o=assets/img_with_q
./build/calibrate_camera assets/img_with_q -c=configs/calibration.yaml
./build/calibrate_robotworld_handeye assets/img_with_q -c=configs/calibration.yaml
./build/calibrate_validate assets/img_with_q -c=configs/calibration.yaml

# 运行 + Foxglove
./build/standard configs/daedalus.yaml
# Foxglove: ws://<机器人IP>:8765
# 布局: foxglove_layouts/dge_debug_layout.json
```

## 关键字段

- `/dge/debug.nis`、`nis_threshold`、`nis_fail_rate`、`residual_norm`
- `/dge/debug.sigma_x/y/z/vx/vy/vz/a/w/rf/rl/hl`
- `/dge/debug.state[0..10]`
- `/dge/debug.ekf_init_source`：1 = Solver 位姿初始化，2 = RobustPnp（v0.32）
- `ekf.process_model`：`cwna`（默认）/ `dwna`（sp25 对照）/ `diag_rate`（旧 rmcs）；v0.33
- `/dge/scene`：`tracked_target` 的 1σ/2σ XY 协方差椭圆
- `/sim/ground_truth`：仿真真值（仅 Daedalus）
