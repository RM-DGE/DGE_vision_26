# DGE 文档索引

| 文档 | 内容 |
|---|---|
| [标定程序.md](./标定程序.md) | 教程：机械零点/杆臂/rpy、QD 风格 URDF 外参与自动 pitch、刚性检查和世界系验收 |
| [EKF调试.md](./EKF调试.md) | 状态/不确定性/一致性三层、Foxglove 字段、NIS/σ 判读、调参流程 |
| [融合说明.md](./融合说明.md) | 同济 sp_vision × 南京理工 rmcs 的融合内容、改动文件、使用方式 |

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
- `/dge/scene`：`tracked_target` 的 1σ/2σ XY 协方差椭圆
- `/sim/ground_truth`：仿真真值（仅 Daedalus）
