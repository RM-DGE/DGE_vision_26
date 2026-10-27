# DGE_vision_26 的 ROS2 通信

本目录实现 DGE 自瞄与导航之间的 ROS2 通信。桥接层 `McuRosBridge` 已实现，
但**当前调试分支没有把它接入 `sentry_DGE`**：`sentry_DGE` 只跑纯自瞄。
通信链路用 `mcu_ros_delay_test` 单独验证。

> 当前调试分支：`src/sentry_DGE.cpp` 里的 ROS 接入已暂时注释，纯自瞄链路
> 不初始化 DDS。桥接代码仍保留在 `io/ros2/mcu_ros_bridge.*`，恢复方式见
> `doc/RobotModel4_v0.40.md`。

核心原则：

- **串口只有一个 owner**：`io::CBoard` 继续独占 `/dev/ttyACM*`；ROS 桥接层只挂
  观察者，不重复打开设备。
- **云台内环不进 ROS**：`sentry_DGE` 默认仍然直接 `cboard.send(command)`；
  `ros2.enable_remote_gimbal_cmd` 只用于验证 `/gimbal/cmd` 往返，默认关闭。
- **IMU 带时间戳**：桥接层把 `steady_clock` 的接收时刻估算成 ROS system time，
  导航端用 `header.stamp` 计算数据年龄，不要直接拿“最新值”当当前姿态。
- **控制类 keep_last(1)**：导航信息、状态、目标都用 depth=1，避免旧消息排队。

## 1. 数据流

```text
                 ┌─────────────────────────────────────┐
                 │ sentry_DGE                          │
                 │                                     │
  CBoard 串口 ──►│ 读线程解析 IMU/状态                 │
                 │   └─ observer ──► McuRosBridge      │
                 │        ├─ /mcu/imu   (sensor_msgs)  │
                 │        └─ /mcu/status(sp_msgs)      │
                 │                                     │
                 │ /enemy_status  ◄── 导航             │
                 │ /autoaim_target ◄── 导航            │
                 │   └─ Decider 过滤/选目标            │
                 │                                     │
                 │ Aimer/Tracker/Shooter               │
                 │   └─ cboard.send() ──► 云台/发射    │
                 │                                     │
                 │ /auto_aim_target_info ──► 导航      │
                 └─────────────────────────────────────┘
                              ▲
                              │ ROS2 DDS
                              ▼
                 ┌─────────────────────────────────────┐
                 │ Nav2 / 行为树 / dge_nav_bridge      │
                 │ 订阅 /mcu/imu、/mcu/status          │
                 │ 发布 /enemy_status、/autoaim_target │
                 └─────────────────────────────────────┘
```

> 上面的数据流是“恢复 ROS 接入后”的目标结构。当前调试分支中
> `sentry_DGE` 不会创建 `McuRosBridge`，所以 `/mcu/imu`、`/mcu/status` 没有发布者。

如果以后需要导航也控制底盘，建议新增 `/chassis/cmd`，由下位机协议增加一个
独立帧；不要和 `/gimbal/cmd` 共用一条语义不清的 `/cmd`。

## 2. 话题

| 方向 | 话题 | 类型 | QoS | 说明 |
|---|---|---|---|---|
| 自瞄 → 导航 | `/mcu/imu` | `sensor_msgs/Imu` | best_effort, depth 5 | 200 Hz 默认；测试可调到 1 kHz |
| 自瞄 → 导航 | `/mcu/status` | `sp_msgs/McuStatus` | reliable, depth 1 | 弹速、模式、在线状态、IMU 年龄 |
| 自瞄 → 导航 | `/auto_aim_target_pos` | `std_msgs/String` | reliable, depth 1 | 兼容旧接口，`x,y,z,id` |
| 自瞄 → 导航 | `/auto_aim_target_info` | `sp_msgs/AutoAimTarget` | reliable, depth 1 | 推荐接口，带 header/速度/yaw |
| 导航 → 自瞄 | `/enemy_status` | `sp_msgs/EnemyStatusMsg` | reliable, depth 1 | 无敌敌方 ID（1-based） |
| 导航 → 自瞄 | `/autoaim_target` | `sp_msgs/AutoaimTargetMsg` | reliable, depth 1 | 优先目标 ID（1-based） |
| 可选 | `/gimbal/cmd` | `sp_msgs/GimbalCmd` | best_effort, depth 1 | 仅 `enable_remote_gimbal_cmd=true` 时写串口 |

ID 约定：`ArmorName + 1`，`one=1 … sentry=6, outpost=7, base=8, not_armor=9`。

后续如果要改“导航 → 自瞄”的控制指令（增加模式、目标、开火权限，或新增
`/nav/control` 话题），按 `doc/RobotModel4_v0.40.md` 的 §2.5 操作即可。

## 3. 编译

ROS2 Humble 的 `rosidl` 无法处理包含中文的路径，所以先复制到 ASCII 路径：

```bash
cd /home/hhh/桌面/DGE_vision_26
./ros2_ws/build.sh
source ~/dge_vision_ros2_ws/install/setup.bash
```

然后编译 DGE：

```bash
cmake -B build -S .
cmake --build build --target sentry_DGE -j
```

如果 `cmake` 输出 `ROS2/sp_msgs not found`，说明没有 source
`~/dge_vision_ros2_ws/install/setup.bash`。此时 `sentry_DGE` 照常编译，
`mcu_ros_delay_test` 不会生成。

> 注意：DGE 的 `sp_msgs` 与 sp25 的 `sp_msgs` 同名。不要同时 source
> `~/sp_vision_ros2_ws` 和 `~/dge_vision_ros2_ws`，否则后 source 的会覆盖前者。
> 只跑 DGE 时用 `~/dge_vision_ros2_ws` 即可。

## 4. 运行

```bash
source /opt/ros/humble/setup.bash
source ~/dge_vision_ros2_ws/install/setup.bash

# 自瞄（真机；当前版本不发布 ROS 话题）
./build/sentry_DGE configs/sentry.yaml

# 只测串口 <-> ROS 桥接（真机）
./build/mcu_ros_delay_test -c=configs/sentry.yaml -d=10 -r=100 -m=real

# 导航侧参考节点（没有完整 Nav2 时联调用）
ros2 run dge_nav_bridge nav_bridge --ros-args \
    -p enemy_ids:="2,5" -p target_ids:="1,3"
```

检查数据：

```bash
ros2 topic hz /mcu/imu
ros2 topic echo /mcu/status
ros2 topic echo /auto_aim_target_info
```

## 5. 延时测试

`tests/mcu_ros_delay_test.cpp` 参考 `detector_video_test.cpp` 的命令行结构，
不需要相机/串口即可跑完整链路：

```bash
cmake --build build --target mcu_ros_delay_test -j
./build/mcu_ros_delay_test -c=configs/sentry.yaml -d=10 -r=100 -m=loopback
```

输出示例（本机 loopback，1 kHz IMU、100 Hz 指令、50 Hz 状态）：

```text
[delay] imu uplink           n=  5001  p50=   0.030 ms  p90=   0.081 ms  p99=   0.167 ms
[delay] status uplink        n=   250  p50=  12.9–15.8 ms  p90=  13.0–15.9 ms  p99=  13.1–16.0 ms
[delay] target uplink        n=    50  p50=   0.024 ms  p90=   0.068 ms  p99=   0.099 ms
[delay] nav -> autoaim       n=    50  p50=   0.055 ms  p90=   0.103 ms  p99=   0.128 ms
[delay] cmd publish->send    n=   500  p50=   0.110 ms  p90=   0.183 ms  p99=   0.349 ms
[stats] cmd_rx=500 cmd_sent=500 cmd_drop=0 drop_rate=0.000%
```

`status` 的十几毫秒不是 DDS 慢，而是 loopback 的 FakeBoard 状态源只有 50 Hz，
固定频率发布时取到的最新样本本身就有 0–20 ms 的年龄。真机状态源约 175 Hz，
发布 50 Hz 时最新样本最多只旧 5.7 ms，实测 p50≈3 ms。

数据年龄和更新间隔要分开看：20 Hz 发布时更新间隔是 50 ms，但每个样本的
`header.stamp` 仍然只旧约 3 ms。

这些数字只包含 **ROS2 DDS + 桥接层回调**，不包含：

- 串口实际收发时间（约 0.5–2 ms，取决于波特率和帧长）；
- 下位机控制周期和云台机械响应；
- 图像采集、YOLO 推理、Tracker/Aimer 计算。

真机串口模式：

```bash
./build/mcu_ros_delay_test -c=configs/sentry.yaml -m=real -d=10
```

没有 MCU 时只能验证“发布 → 桥接回调 → `CBoard::send()` 入队”这一跳；
`CBoard::send()` 只是把数据放进串口写线程队列，真正写入串口还会多约 1 ms。

## 6. 延时预算

按 14 rad/s 小陀螺估算：

- 1 ms ≈ 0.8°；
- 3 m 处小装甲半角约 1.29°；
- 5 m 处小装甲半角约 0.77°。

因此：

- ROS 本机小消息 0.1–0.5 ms 可以接受，但必须计入 `Aimer` 的
  `high_speed_delay_time` / `low_speed_delay_time`；
- 串口 + 下位机控制约 1–3 ms，是主要固定延时；
- 真正不能接受的是**抖动**和**旧指令排队**，所以：
  - QoS `keep_last(1)`；
  - 桥接层拒绝超过 `max_cmd_age_ms` 的指令；
  - IMU 用时间戳插值，不直接用最新值；
  - 下位机做超时保护，导航 200 ms 没新指令就减速/停车。

如果实测 p99 抖动大，优先把云台火控放到下位机（参考辽宁 COD 的方案），
视觉只发带时间戳的 yaw/pitch 和目标角速度。

## 7. 已知限制

- 当前串口协议只有 IMU 四元数和状态帧，没有角速度、线加速度、里程计；
  `/mcu/imu` 的 `angular_velocity` / `linear_acceleration` 按 REP-145 置为
  “未知”（协方差 -1），导航不要依赖这两项。
- 没有 `/mcu/odom`。导航的轮式里程计应由底盘 MCU 另发，或扩展串口协议。
- `0x5A` 雷达包目前被 `SerialPortMultithread` 丢弃；如果导航需要这路数据，
  应新增一个观察者/话题，而不是让导航直接读串口。
- `dge_nav_bridge` 只是联调参考，真实导航应把 `enemy_ids` / `target_ids`
  换成行为树输出。
