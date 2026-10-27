# RobotModel4 v0.40 修改记录（ROS2 桥接与时间戳）

> 基于 v0.33。范围：DGE 与导航/下位机的 ROS2 桥接、CBoard 观察者接口、
> IMU/状态上行、导航目标下行、云台指令可选回环，以及 IMU/相机时间戳的
> 测试口径。**EKF 的预测、观测、关联和 NIS 数学逻辑未改。**

> **当前调试分支状态**
>
> - `src/sentry_DGE.cpp` 已移除 ROS 接入，只保留纯自瞄路径，不初始化 DDS；
> - `io/ros2/mcu_ros_bridge.*`、`ros2_ws/`、`tests/mcu_ros_delay_test.cpp` 仍保留；
> - 恢复 ROS 时不要在 `sentry_DGE.cpp` 里重新堆 `#ifdef`，建议用独立 adapter；
> - `src` 只做参数解析、模块构造和主循环；全向调试画面在
>   `tasks/omniperception/visualizer.cpp`。

---

## 1. 本版解决什么问题

v0.33 之前，DGE 的主循环只关心自瞄：

- 串口由 `io::CBoard` 独占；
- 导航需要的数据没有统一的 ROS 出口；
- 自瞄和导航各自解析串口会破坏“单 owner”；
- `imu_at(timestamp - 1ms)` 的 1 ms 只是经验值，没有测试方法；
- 下位机 IMU 的实际频率、串口实际延时没有量化的入口。

v0.40 把“串口数据怎么给导航、导航信息怎么给自瞄、时间戳怎么对齐”固定下来：

```text
CBoard 读线程
  ├─ imu_at 队列（自瞄原有路径）
  └─ observer（新增）
       └─ McuRosBridge
            ├─ /mcu/imu
            ├─ /mcu/status
            ├─ /auto_aim_target_info
            └─ /enemy_status、/autoaim_target（订阅）
```

串口仍然只有一个 owner；ROS 只负责信息共享，默认不进入云台控制内环。

---

## 2. 接口变化

### 2.1 `CBoardBase`

新增观察者接口，读线程只负责把样本交给回调：

```cpp
struct ImuSample
{
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point timestamp;
};

struct StatusSample
{
  double bullet_speed;
  int mode;
  int shoot_mode;
  double ft_angle;
  std::chrono::steady_clock::time_point timestamp;
};

virtual void set_imu_observer(ImuObserver observer);
virtual void set_status_observer(StatusObserver observer);
```

约定：

- 回调里只做 mailbox 更新，**禁止直接 publish**；
- `timestamp` 是串口帧解析完成时的 `steady_clock`，不是 MCU 采样时刻；
- `DaedalusCBoard` 可以不实现观察者，仿真下桥接层只是没有上行数据。

### 2.2 `McuRosBridge`

新增 `io/ros2/mcu_ros_bridge.hpp/.cpp`：

| 方向 | 话题 | 类型 | 默认频率 |
|---|---|---|---:|
| 自瞄 → 导航 | `/mcu/imu` | `sensor_msgs/Imu` | 200 Hz 上限 |
| 自瞄 → 导航 | `/mcu/status` | `sp_msgs/McuStatus` | 20 Hz |
| 自瞄 → 导航 | `/auto_aim_target_pos` | `std_msgs/String` | 每帧 |
| 自瞄 → 导航 | `/auto_aim_target_info` | `sp_msgs/AutoAimTarget` | 每帧 |
| 导航 → 自瞄 | `/enemy_status` | `sp_msgs/EnemyStatusMsg` | 导航决定 |
| 导航 → 自瞄 | `/autoaim_target` | `sp_msgs/AutoaimTargetMsg` | 导航决定 |
| 可选 | `/gimbal/cmd` | `sp_msgs/GimbalCmd` | 100 Hz |

实现要点：

- IMU：事件驱动，发布最新样本；超过 `imu_publish_rate` 时降采样；
- 状态：固定频率发布最新值，不跟随源帧率；
- 导航信息：latest-value + 1.5 s 超时；
- 云台指令：默认不订阅；`enable_remote_gimbal_cmd=true` 时才写串口；
- 所有控制类 QoS 使用 `keep_last(1)`，避免旧指令排队。

### 2.3 `sentry_DGE`（原设计，当前已停用）

原设计：

- 启动时按 `ros2.enabled` 创建桥接层；
- `get_invincible_armor()` 在 `armor_filter()` 前更新；
- `get_auto_aim_target()` 在过滤后裁剪；
- 云台指令默认仍直接 `cboard.send(command)`；
- 目标回传统一走 `publish_target_info()`，避免主循环堆 ROS 细节。

当前调试分支已把 `sentry_DGE` 里的 ROS 接入移除，先保证纯自瞄链路可运行。
上面的 `publish_target_info()`、导航目标注入和 `/gimbal/cmd` 分支现在都不在主程序里；
恢复时参考 `io/ros2/mcu_ros_bridge.*`，并建议用独立的 ROS adapter 文件。

### 2.4 ROS2 包

新增 `ros2_ws/src/sp_msgs`：

- `McuStatus.msg`
- `GimbalCmd.msg`
- `EnemyStatusMsg.msg`
- `AutoaimTargetMsg.msg`
- `AutoAimTarget.msg`

新增 `ros2_ws/src/dge_nav_bridge`：

- 订阅 `/mcu/imu`、`/mcu/status`；
- 发布 `/enemy_status`、`/autoaim_target`；
- 仅作为导航组联调参考，真实决策由 Nav2/行为树给出。

### 2.5 后续扩展：导航发送、自瞄接收

这一节是给后续改“导航 → 自瞄”控制指令用的。当前代码只实现了
`/enemy_status` 和 `/autoaim_target`；下面的 `NavControl` 是扩展模板，不是已实现接口。

先定一条原则：

> **导航发给自瞄的应该是“意图”，不是直接覆盖云台/开火。**
>
> - 可以发：打谁、模式、是否允许开火、目标优先级、忽略列表；
> - 不建议发：直接 `yaw/pitch` 覆盖 `Aimer`、直接 `shoot`、1 kHz 高频云台指令。
>
> 云台和火控的 owner 仍然是自瞄；导航只改变“选谁、打不打”的上层决策。

#### 2.5.1 当前接口

| 方向 | 话题 | 类型 | 自瞄侧入口 |
|---|---|---|---|
| 导航 → 自瞄 | `/enemy_status` | `sp_msgs/EnemyStatusMsg` | `decider.get_invincible_armor()` |
| 导航 → 自瞄 | `/autoaim_target` | `sp_msgs/AutoaimTargetMsg` | `decider.get_auto_aim_target()` |

两条都是 latest-value + 1.5 s 超时；超时后按“没有导航信息”处理。

#### 2.5.2 方案 A：给现有消息加字段（改动最小）

适合只增加“目标选择/模式”这类低频信息。以 `/autoaim_target` 为例：

```text
# ros2_ws/src/sp_msgs/msg/AutoaimTargetMsg.msg
builtin_interfaces/Time timestamp
int8[] target_ids
uint8 mode          # 0=不改变，1=巡逻，2=追击，3=强制目标
uint8 fire_policy   # 0=不改变，1=允许开火，2=禁止开火
```

修改步骤：

1. 改 `AutoaimTargetMsg.msg`；
2. 重新编译消息：

   ```bash
   ./ros2_ws/build.sh
   source ~/dge_vision_ros2_ws/install/setup.bash
   ```

3. 导航侧发布时填新字段；
4. `McuRosBridge` 的 `autoaim_target_callback()` 保存新字段；
5. 增加一个带超时的 getter；
6. `sentry_DGE` 主循环读取并应用；
7. 用 `ros2 topic pub` / `mcu_ros_delay_test` 验证。

自瞄侧建议暴露一个结构体，而不是继续加一堆散落的 getter：

```cpp
struct NavTarget
{
  std::vector<int8_t> ids;
  uint8_t mode = 0;
  uint8_t fire_policy = 0;
  bool valid = false;
};

NavTarget nav_target();  // 内部做 1.5 s 超时
```

回调只做 mailbox 更新：

```cpp
void McuRosBridge::autoaim_target_callback(
  const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(nav_mutex_);
  autoaim_target_ids_ = msg->target_ids;
  autoaim_target_mode_ = msg->mode;
  autoaim_target_fire_policy_ = msg->fire_policy;
  autoaim_target_time_ = std::chrono::steady_clock::now();
}
```

`fire_policy` 用 `0=不改变` 而不是 `bool`，是为了兼容旧导航：旧导航不发这个字段时，默认值 0 不会把自瞄的开火权限关掉。

#### 2.5.3 方案 B：新增话题（推荐用于复杂控制）

如果导航要发的信息超过“目标 ID + 模式”，建议独立话题，不要继续膨胀 `/autoaim_target`：

```text
# ros2_ws/src/sp_msgs/msg/NavControl.msg
std_msgs/Header header
uint32 seq
uint8 mode          # 0=idle, 1=patrol, 2=chase, 3=forced_target
bool enable
int8 target_id
uint8 fire_policy   # 0=不改变, 1=允许, 2=禁止
float32 yaw_hint    # 可选，仅作方向建议
float32 pitch_hint  # 可选，仅作方向建议
```

修改步骤：

1. 新增 `NavControl.msg`；
2. 在 `ros2_ws/src/sp_msgs/CMakeLists.txt` 的 `rosidl_generate_interfaces()` 里加文件；
3. 重新 `./ros2_ws/build.sh`；
4. `configs/sentry.yaml` 增加话题名：

   ```yaml
   ros2:
     nav_control_topic: "/nav/control"
   ```

5. `McuRosBridge::Config` 增加 `nav_control_topic`；
6. 构造函数里创建订阅：

   ```cpp
   nav_control_sub_ = node_->create_subscription<sp_msgs::msg::NavControl>(
     config_.nav_control_topic, rclcpp::QoS(1).reliable(),
     [this](sp_msgs::msg::NavControl::SharedPtr msg) { nav_control_callback(msg); });
   ```

7. 增加成员、回调和带超时的 getter：

   ```cpp
   struct NavControl
   {
     bool valid = false;
     uint8_t mode = 0;
     bool enable = false;
     int8_t target_id = -1;
     uint8_t fire_policy = 0;
     double yaw_hint = 0.0;
     double pitch_hint = 0.0;
     uint32_t seq = 0;
     std::chrono::steady_clock::time_point stamp{};
   };

   NavControl nav_control();
   ```

8. `sentry_DGE` 主循环里读取：

   ```cpp
   #ifdef DGE_HAS_ROS2
   if (ros_bridge) {
     const auto nav = ros_bridge->nav_control();
     if (nav.valid && nav.enable) {
       // 只改模式/目标/开火权限；
       // 不要把 yaw_hint/pitch_hint 直接覆盖 Aimer 输出。
     }
   }
   #endif
   ```

9. 导航侧发布：

   ```python
   msg = NavControl()
   msg.header.stamp = self.get_clock().now().to_msg()
   msg.seq = self.seq
   msg.mode = 2
   msg.enable = True
   msg.target_id = 3
   msg.fire_policy = 1
   self.nav_control_pub.publish(msg)
   ```

#### 2.5.4 修改位置速查

| 要改什么 | 文件 |
|---|---|
| 消息字段 | `ros2_ws/src/sp_msgs/msg/*.msg` |
| 消息编译 | `ros2_ws/src/sp_msgs/CMakeLists.txt` |
| 重新生成 | `./ros2_ws/build.sh` |
| 话题名/超时 | `configs/sentry.yaml` 的 `ros2:` |
| 导航侧发布 | `ros2_ws/src/dge_nav_bridge/.../nav_bridge.py` 或 Nav2 行为树 |
| 桥接订阅/回调 | `io/ros2/mcu_ros_bridge.hpp/.cpp` |
| 自瞄使用 | `src/sentry_DGE.cpp` |
| 延时/联调测试 | `tests/mcu_ros_delay_test.cpp`、`ros2 topic pub/echo` |

#### 2.5.5 QoS、超时和兼容性

- 导航 → 自瞄的控制指令用 `reliable + keep_last(1)` 或
  `best_effort + keep_last(1)`，频率 10–50 Hz；
- 每条消息带 `header.stamp` 和 `seq`；
- 自瞄侧统一做 1.5 s 超时，超时后回到默认行为，不要继续使用旧指令；
- 回调里只更新 mailbox，不直接改 `Aimer`、不直接 `cboard.send()`；
- 新字段尽量加在消息末尾，不要改旧字段语义；
- `fire_policy`、`mode` 这类字段用“0=不改变”的枚举，避免旧导航默认值误关火；
- ID 仍使用 1-based（`ArmorName + 1`），并校验 1–9；
- 如果将来确实要导航直接给云台角度，单独定义 `NavGimbalCmd`，并加模式仲裁；
  不要和 `/autoaim_target` 混在一起。

#### 2.5.6 联调命令

```bash
# 看消息定义
ros2 interface show sp_msgs/msg/NavControl

# 手动发一条
ros2 topic pub --once /nav/control sp_msgs/msg/NavControl \
  "{header: {frame_id: map}, seq: 1, mode: 2, enable: true, target_id: 3, fire_policy: 1}"

# 看自瞄是否收到
ros2 topic echo /nav/control
```

在 `sentry_DGE` 里建议加一条低频日志：

```cpp
tools::logger()->info(
  "[NAV] mode={} target={} fire={} age={:.1f} ms",
  nav.mode, int(nav.target_id), int(nav.fire_policy), age_ms);
```

这样导航组改指令时，能直接确认自瞄收到的是哪一版语义。

---

## 3. 时间戳口径

### 3.1 当前两个时间戳都不是“曝光时刻”

| 数据 | 当前时间戳 | 含义 |
|---|---|---|
| 图像 | `HikRobot` 在 `MV_CC_GetImageBuffer()` 返回后取 `steady_clock::now()` | 主机收到整帧 |
| IMU | `CBoard::callback()` 解析完一帧后取 `steady_clock::now()` | 主机收到 IMU 帧 |

因此：

```cpp
Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
```

中的 `1ms` 只能作为初值，不能当作标定结果。图像侧包含曝光、读出、USB 传输和
采集线程的 1 ms sleep；IMU 侧包含串口传输和读线程调度。两者不是同一个物理时刻。

### 3.2 延时预算

本机 loopback（1 kHz IMU、100 Hz 指令、50 Hz 状态）实测：

| 项目 | p50 | p99 |
|---|---:|---:|
| IMU 上行 | 0.030 ms | 0.167 ms |
| 状态上行 | 约 13–16 ms* | 约 16 ms* |
| 目标上行 | 0.024 ms | 0.099 ms |
| 导航 → 自瞄 | 0.055 ms | 0.128 ms |
| 指令发布 → `CBoard::send()` | 0.110 ms | 0.349 ms |

真机 `-m=real` 实测（115200、IMU 约 165 Hz）：

| 项目 | p50 | p99 |
|---|---:|---:|
| IMU 上行 | 0.16–0.21 ms | 0.36–0.41 ms |
| 状态上行 | 约 3 ms | 约 5.6 ms |
| 指令发布 → `CBoard::send()` | 0.09–0.16 ms | 0.27–0.31 ms |

注意：

- 状态的延时取决于“源帧率”和“发布频率”的较小者：真机状态源约 175 Hz、
  发布 50 Hz 时，每次发布取到的最新样本最多旧 5.7 ms，所以 p50≈3 ms；
  loopback 的 FakeBoard 状态只有 50 Hz，故 p50≈13–16 ms（*）。
- 20 Hz 发布时，状态的“数据年龄”仍约 3 ms，但“更新间隔”是 50 ms；
  导航应把更新间隔和 header.stamp 年龄分开看。
- 这些数字不包含串口线上时间；8 字节控制帧在 115200 8N1 下约 0.69 ms；
- 不包含图像采集、YOLO、Tracker/Aimer、下位机云台响应；
- IMU 约 165 Hz 时，两帧间隔约 6.05 ms；14 rad/s 下约 4.85°。

---

## 4. 测试

### 4.1 通信延时

```bash
./ros2_ws/build.sh
source ~/dge_vision_ros2_ws/install/setup.bash

cmake -B build -S .
cmake --build build --target mcu_ros_delay_test -j

# 无硬件回环
./build/mcu_ros_delay_test -c=configs/sentry.yaml -d=10 -r=100 -m=loopback

# 真机串口
./build/mcu_ros_delay_test -c=configs/sentry.yaml -d=10 -r=100 -m=real
```

输出项：

```text
imu uplink
status uplink
target uplink
nav -> autoaim
cmd publish->send
board.send() call
cmd_drop / last_cmd_age
```

### 4.2 IMU/相机时间戳对齐

不要反复调用 `imu_at()` 做扫描，它会消费 IMU 队列。正确做法：

1. 用 `set_imu_observer()` 把 IMU 样本录到 vector；
2. 相机每帧记录 `t_img` 和 PnP 结果；
3. 离线在 IMU 样本上对候选延时 `d` 插值；
4. 用固定棋盘格/静止装甲板的 PnP 姿态作为图像侧真值；
5. 找到使角误差最小的 `d`，写回 `imu_delay_ms`。

推荐实验：

- 棋盘格固定，云台匀速 yaw 转动；
- 速度取 ±2、±5、±10 rad/s；
- 曝光 0.5–1 ms，避免运动模糊；
- 每个速度录 10 s，扫 `d ∈ [-20, +20] ms`；
- 最优 `d` 在不同速度/方向下应一致；
- 同时记录插值比例 `k`，大部分应在 `[0,1]`。

如果 `k > 1` 经常出现，说明请求时刻比最新 IMU 还新，当前 `imu_at()` 会外推；
建议加 `k = std::clamp(k, 0.0, 1.0)`，并在数据过旧时标记 invalid。

---

## 5. 修改文件

| 文件 | 内容 |
|---|---|
| `io/cboard.hpp/.cpp` | 新增 `ImuSample`/`StatusSample`、observer、`shoot_mode` |
| `io/ros2/mcu_ros_bridge.hpp/.cpp` | 新增 ROS2 桥接层、QoS、超时、统计 |
| `src/sentry_DGE.cpp` | 接入桥接层、导航目标过滤、目标回传；当前调试分支已暂时关闭 ROS |
| `tasks/omniperception/visualizer.cpp` | 全向雷达/USB 调试画面，绘制 `DetectionResult`/`DebugFrame` |
| `io/CMakeLists.txt` | 可选 `io_ros2` 目标 |
| `CMakeLists.txt` | 可选 `io_ros2` 和 `mcu_ros_delay_test`；`sentry_DGE` 当前不链接 ROS |
| `configs/sentry.yaml` | 新增 `ros2:` 配置块 |
| `ros2_ws/src/sp_msgs` | 新增/补齐消息定义 |
| `ros2_ws/src/dge_nav_bridge` | 导航侧参考节点 |
| `tests/mcu_ros_delay_test.cpp` | 通信链路延时测试 |
| `ros2_ws/README.md` | 话题、编译、测试、限制 |

---

## 6. 已知限制

1. 当前串口协议只有 IMU 四元数和状态帧，没有角速度、线加速度、里程计。
   `/mcu/imu` 的 `angular_velocity` / `linear_acceleration` 按 REP-145 标为未知。
2. 没有 `/mcu/odom`。导航的 `odom -> base_link` 需要底盘 MCU 另发里程计，
   或扩展串口协议。
3. 没有 `/chassis/cmd`。导航控制底盘的帧尚未定义，`ControlDataFrame` 目前只有云台。
4. `0x5A` 雷达包仍被 `SerialPortMultithread` 丢弃；高频激光/点云不要走 115200 串口。
5. 真机 IMU 约 165 Hz，115200 波特率；若要 1 kHz IMU，需要提高波特率并调整读线程。
6. 海康相机时间戳是主机收帧时间，不是曝光时间；`-1ms` 需要实测。
7. `CBoard::imu_at()` 的插值比例 `k` 尚未 clamp，极端情况下会外推。
8. `sentry_DGE` 仍是串口单 owner。若导航也依赖底盘控制，建议把 `mcu_bridge`
   独立成进程，避免自瞄崩溃导致导航失去下位机链路。

---

## 7. 验收清单

- [ ] `cmake -B build -S .` 能找到 `sp_msgs`，`io_ros2` 被编译；
- [ ] 不 source ROS 时 `sentry_DGE` 仍能编译；
- [ ] `ros2 topic hz /mcu/imu` 与下位机实际频率一致；
- [ ] `/mcu/status` 按配置频率发布，掉线后 `online=false`；
- [ ] 导航发布 `/enemy_status` 后，自瞄能过滤无敌目标；
- [ ] 导航发布 `/autoaim_target` 后，自瞄只保留指定目标；
- [ ] `/auto_aim_target_info` 的 `header.stamp` 接近图像时间；
- [ ] `mcu_ros_delay_test -m=loopback` 和 `-m=real` 均无 `cmd_drop`；
- [ ] 用棋盘格扫 `imu_delay_ms`，最优延时在多次实验中稳定。

---

## 8. 下一版

v0.41 建议优先补：

1. `/mcu/odom` 与 `odom -> base_link`；
2. `/chassis/cmd` 串口帧、seq、时间戳和 MCU 看门狗；
3. IMU 500–1000 Hz 与 460800/921600 波特率；
4. `CBoard::imu_at()` 的 `k` clamp 和过期数据 invalid 标记；
5. 完整链路测量：图像时间戳 → 检测 → Aimer → 串口写完成。
