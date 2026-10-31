# rm_aim_sim 调试器适配设计

日期：2026-10-31  
目标分支：`codex/rm_aim_sim`（基于 `test` 提交 `feaba15`）  
上游调试器：https://github.com/ForgetWW520/rm_aim_sim

## 1. 目标

在 WSL 中独立运行 `rm_aim_sim`，让本 DGE 自瞄工程通过该调试器提供的同机 IPC 完成真实像素闭环：读取原始相机帧及同帧姿态，执行现有检测、PnP、跟踪、预测与瞄准逻辑，再向调试器写入云台和发射指令，并以调试器的命中统计作为端到端验收结果。

新适配只存在于 `codex/rm_aim_sim` 分支。原 `test` 工作区、真机入口和未提交内容不受影响。

## 2. 范围

### 包含

- 在 WSL 中以独立仓库运行 `rm_aim_sim`，不复制 Godot 工程到本仓库。
- 新增 `rm_aim_sim` 专用 C++ IPC 客户端。
- 新增独立的 `auto_aim_sim` 可执行程序和仿真配置。
- 复用现有 detector、solver、tracker、aimer 和 Foxglove 调试能力。
- 建立图像、姿态、标定、状态、控制和失联保护的完整链路。
- 增加协议单元测试及 WSL 像素闭环验收脚本/说明。
- 从新分支的构建、入口和文档中移除旧 Bevy/Daedalus 专用代码。

### 不包含

- 修改或捆绑 `rm_aim_sim` 上游 Godot 源码。
- 让真值关键点替代检测结果。
- 模拟工业相机、串口、CAN、ROS 或真实底盘硬件。
- 重写现有检测、跟踪、预测和弹道算法。
- 删除历史 RobotModel 文档中作为变更记录存在的旧名词。

## 3. 仓库与分支隔离

- 源分支固定为 `test` 的已提交基线 `feaba15`。
- 功能分支为 `codex/rm_aim_sim`。
- 功能分支在独立 Git worktree 中开发，禁止在用户当前 `test` 工作区切换分支或覆盖文件。
- 当前 `test` 工作区内的旧 Bevy/Daedalus 未提交修改、临时 Rust 文件和截图均不带入新分支。
- 所有删除均按明确文件逐个执行；不批量删除目录。

## 4. 推荐架构

### 4.1 组件边界

`io/rm_aim_sim/` 提供一个聚合客户端，内部再分为可单测的小组件。实现锁定上游提交 `3895971a25e7712286395840b3c4cea9dffbc27c` 的协议 v2：

- `Protocol`：定义 JSON 字段、像素格式、序号和协议校验。
- `FrameReader`：按上游握手读取 `frame.json` 指向的 `frame-*.raw`，验证文件大小和序号，输出 BGR `cv::Mat`、同帧姿态、相机参数和时间戳。
- `StateReader`：读取 `state.json`，提供实际云台姿态、弹速、统计、心跳/更新时间及只用于诊断的 truth。
- `CommandWriter`：将 DGE 的控制结果转换为协议字段，通过“临时文件 + 原子重命名”发布 `command.json`。
- `ClockMapper`：在每个 `(publisher_id, session_id)` epoch 内把 `sim_time` 映射到本进程 `std::chrono::steady_clock`；暂停、重置或发布者重启时废弃旧映射。
- `RmAimSimClient`：管理通道路径、生命周期、断流状态及上述组件。

上层 `src/auto_aim_sim.cpp` 直接使用该客户端，不把仿真协议重新塞入真机 `Camera`/`CBoard` 工厂。

### 4.2 数据流

```text
rm_aim_sim
  state.json
      | 读取 publisher/session、标定和实际姿态
      v
RmAimSimClient --原子写 request.json--> rm_aim_sim
      ^                                  |
      |--------- frame.json/raw ---------|
          |
          v
RmAimSimClient -> BGR 图像 + 同帧姿态 + steady_clock 时间戳
          |
          v
Detector -> Solver -> Tracker -> Aimer
          |
          v
RmAimSimClient -> command.json -> Godot 云台/发射

state.json -> 实际姿态/弹速/统计/truth（truth 仅诊断与评分）
```

### 4.3 入口与配置

- 新目标：`auto_aim_sim`。
- 新配置：`configs/rm_aim_sim.yaml`。
- `[sim]` 配置至少包含 `channel`、IPC 根目录、帧等待超时、状态超时和是否发布诊断 truth。
- 相机内参优先采用每帧元数据或协议提供的标定；静态 YAML 只作为显式校验/回退，不静默覆盖协议值。
- 启动命令与上游保持两个终端：先启动 `rm_aim_sim --remote`，再运行 `auto_aim_sim`。

## 5. 协议规则

实现以上游提交 `3895971a25e7712286395840b3c4cea9dffbc27c` 的 `docs/protocol.md` 和 `examples/cpp/sim_client_demo.cpp` 为协议依据，不复用旧 Talos/Daedalus 的二进制结构体布局。

- 默认通道目录为 `/dev/shm/rm_aim_sim_<channel>/`。
- 图像采用协议声明的无压缩 RGB/RGBA；读取后显式转换为 OpenCV BGR。
- 必须校验宽、高、stride/通道数、格式、原始文件长度和帧序号。
- 相机是请求式单帧握手：读取新鲜 `state.json` 后，客户端生成唯一 `request_id` 并原子写 `request.json`；仅接受 `publisher_id/request_id` 同时匹配的 `frame.json`，复制完 `frame-<request_id>.raw` 后才允许下一次请求。
- `frame.json` 的 `publisher_id/session_id`、相机标定和姿态必须作为同帧元数据整体交给算法，不能用更新的 `state.json` 姿态替换。
- 命令发布采用同目录临时文件并原子替换目标文件；临时文件名包含进程标识，避免覆盖别的客户端临时文件。
- 单通道只允许一个 DGE 控制端；通过 `openat(..., O_NOFOLLOW)` 打开 `client.lock` 并持有非阻塞 `flock(LOCK_EX|LOCK_NB)`，失败即报错，不抢占。
- 打开 IPC 目录后以 `fstat` 校验当前 UID 和 `0700` 权限，所有后续文件均相对目录 fd 使用 `openat/renameat`，拒绝符号链接。
- 不依赖文件系统事件才能正确工作；轮询路径保留有界等待和退出检查，确保 WSL/tmpfs 环境稳定。

## 6. 坐标、姿态与时间

- 图像坐标遵循 OpenCV：原点左上，`+x` 向右，`+y` 向下。
- 相机坐标遵循协议声明；只在适配层做一次明确、可测试的轴变换。
- 不沿用 `daedalus.yaw_sign`、`pitch_sign`、`imu_left/right`、固定 `-90°` 或经验 trim 等旧协议补丁。
- DGE 指令的绝对/相对语义、弧度/角度和 yaw/pitch 正方向由协议测试固定；转换函数必须为纯函数并覆盖往返测试。
- 图像、姿态和标定必须来自同一帧序号。无法得到同帧姿态时丢弃该帧，不用最新姿态冒充。
- `ClockMapper` 以每个 epoch 的首个有效帧建立 `sim_time -> steady_clock` 偏移；仿真暂停时不伪造时间前进，`publisher_id/session_id` 改变或时间倒退时重新建基并重建 Tracker/Aimer 状态。
- `system_clock`、Godot 仿真时间和 DGE `steady_clock` 不直接混算。

## 7. 安全与错误处理

以下情况进入安全状态：

- 仿真器目录不存在或协议版本不兼容。
- 帧/状态超过配置超时。
- 序号倒退、时间回退、JSON 不完整、像素文件长度错误。
- 分辨率、格式或标定在运行中改变且上层尚未完成重配置。
- 命令写入或原子替换失败。

安全状态行为：

- 立即发布或保持 `control=false`、`fire=false` 的安全命令（以协议支持的失控语义为准）。
- 清除未确认的开火请求，不复用上一帧射击许可。
- 当前帧不进入 detector/tracker/aimer。
- 对重复错误限频记录；恢复后记录新的 epoch，并重置依赖连续时间的跟踪状态。
- 进程退出前尽力发布一次停火命令；该动作失败只记录，不阻塞退出。

## 8. 真值隔离

- detector、PnP、tracker、solver 和 aimer 的输入只能来自像素检测及同帧姿态。
- `truth` 只能进入独立的诊断/评分对象，用于 Foxglove 对照、误差计算和测试断言。
- 通过类型和调用边界避免 truth 对象被转换为 `Armor`/`Target` 后进入算法主链。
- 默认运行日志明确标注“pixel closed loop”；启用 truth 可视化不改变控制结果。

## 9. 旧代码清理

新分支将逐文件处理以下旧 Bevy/Daedalus 专用内容：

- 从 CMake 移除 `io/Daedalus/daedalus_camera.cpp`、旧测试和 include 路径。
- 从 `io/camera.*` 移除 `camera_name: daedalus` 分支以及仿真 truth 泄漏。
- 从 `src/standard.cpp`、`src/sentry_DGE.cpp` 移除 `DaedalusCBoard` 选择逻辑，使真机入口恢复为真机实现。
- 删除明确无其他使用者的 `io/Daedalus/*` 文件、`configs/daedalus.yaml` 及旧 Bevy 专用测试。
- 将当前使用说明、Foxglove 说明和 EKF 调试入口更新为 `rm_aim_sim`；历史版本文档保留，不篡改历史记录。
- `CameraBase` 保留为海康相机采集接口，但删除 `ground_truth()`；`CBoardBase` 保留为 ROS2 桥接和延时测试的注入接口，但删除全部 Daedalus 语义；`SimTargetTruth` 移入新仿真命名空间。

清理的判断标准是全仓库引用分析和成功构建，不以文件名猜测。禁止批量删除目录。

## 10. 测试与验收

### 10.1 单元测试

- JSON 必填字段、版本、非法类型和缺失字段。
- RGB/RGBA 到 BGR 转换及错误像素长度。
- 帧序号去重、跳变、倒退和仿真重置。
- 同帧图像/姿态配对。
- 仿真时间到 `steady_clock` 的映射、暂停和重置。
- yaw/pitch、四元数、单位与轴方向转换。
- command JSON 内容及原子发布，不残留对外可见的半写文件。
- 断流、状态超时、写入失败时始终停火。
- truth 不进入控制链的接口级测试。

测试使用临时目录模拟 `/dev/shm`，不要求启动 Godot。

### 10.2 构建回归

- 在 WSL/Ubuntu 22.04 环境完成 CMake configure 和 build。
- `standard`、`sentry_DGE` 和 `auto_aim_sim` 均可构建。
- 现有可运行测试与新增测试全部通过。
- `rg` 检查活跃构建、配置和当前文档不再引用旧 Daedalus/Bevy 接口；允许历史变更文档保留旧名称。

### 10.3 端到端验收

1. 独立克隆并启动锁定版本的 `rm_aim_sim`，开启 `--remote`。
2. 运行 `auto_aim_sim configs/rm_aim_sim.yaml`。
3. 确认连续接收原始像素帧和同帧姿态，检测结果来自图像。
4. 确认云台跟随、发射和命中统计形成闭环。
5. 暂停/关闭仿真器，确认在超时窗口内停火且进程给出明确诊断。
6. 恢复或重启仿真器，确认新的时间 epoch 不复用旧跟踪状态。
7. 静止靶、平移靶和旋转靶各运行一轮，导出统计并记录上游提交、DGE 提交和配置。

## 11. 具体实现

### 11.1 文件与类型

- `io/rm_aim_sim/protocol.hpp/.cpp`
  - 定义 `SimContext`、`SimCameraModel`、`SimState`、`SimFrame`、`SimCommand` 和 `SimTruth`。
  - `parse_state(json)` 与 `parse_frame(json)` 完成必填字段、有限数值、尺寸、标定数组和协议版本校验。
  - `make_command_json(state, seq, command)` 复制当前 `publisher_id/session_id`，把 `io::Command` 转为绝对 yaw/pitch 弧度命令。
- `io/rm_aim_sim/local_client.hpp/.cpp`
  - 构造函数验证 channel 正则 `[A-Za-z0-9_-]{1,80}`、目录所有权/权限并获得 `client.lock`。
  - `poll_state()` 只在 `publication` 增长时更新接收单调时刻；重复状态不会刷新 watchdog。
  - `request_frame()` 每次生成 `<pid>-<启动token>-<递增号>`，原子写 `request.json`。
  - `try_read_frame()` 校验上下文、文件名、编码、尺寸与字节数，用 `read` 完整复制后执行 RGB/RGBA→BGR。
  - `send()` 从 `state.ipc.last_seq + 1` 同步严格递增序号；paused、IPC 未启用或状态不新鲜时强制 `control=false/fire=false`。
  - 析构和正常退出路径尽力发送停控停火，再释放 `flock`。
- `io/rm_aim_sim/clock_mapper.hpp/.cpp`
  - `map(context, sim_time, received_at)` 返回稳定的 `steady_clock::time_point`。
  - context 变化、仿真时间倒退或暂停恢复返回 `epoch_changed=true`，上层据此丢弃旧请求并重建 Tracker/Aimer。
- `src/auto_aim_sim.cpp`
  - 以单线程主循环按 `poll_state → request_frame → try_read_frame → detect → set quaternion → track → aim/shoot → send` 顺序执行。
  - 图像和 `frame.metadata.gimbal.quaternion_wxyz` 同时消费；四元数直接构造 `Eigen::Quaterniond(w,x,y,z)`，不套旧 Daedalus 轴变换。
  - 协议和现有 `Aimer` 的 `io::Command::pitch` 均为向下为正：`Aimer` 对向上的弹道角显式取负，因此 `make_command_json` 直接传递 pitch，不再反号；静止高/低靶测试固定该约定。
  - `frame.camera.K/distortion/R_camera2gimbal/t_camera2gimbal` 与 `configs/rm_aim_sim.yaml` 启动值逐项比较；不一致时停控并报出字段差异，避免 Solver 在运行中静默换内参。
  - `truth` 只转换为 `SimTruth` 并交给 Foxglove 诊断，不生成 `Armor` 或 `Target`。

### 11.2 每帧执行顺序

1. 读取并解析 `state.json`；首次连接用 `wall_unix_sec` 拒绝超过 250 ms 的遗留状态，随后用 `steady_clock` 和 `publication` 维护 200 ms watchdog。
2. 检测 `publisher_id/session_id`、paused、IPC enabled、车型/阵营或相机标定变化；变化时取消旧请求、发送停控、重建时间 epoch 和 Tracker/Aimer。
3. 在没有未完成请求时原子发布 `request.json`。
4. 轮询匹配的 `frame.json`；5 s 没有图像时提示必须使用图形模式而非 `--headless`，并保持停控停火。
5. 使用 `openat` 打开精确的 `frame-<request_id>.raw`，校验普通文件与长度，复制到自有 `cv::Mat`，再转换成 BGR。
6. 用同一 `SimFrame` 的四元数和映射时间更新 Solver/Tracker；运行 detector、tracker、aimer、shooter。
7. 将结果转换为协议绝对角命令；只有状态新鲜、session 未变、外部控制开启且本帧有效时才允许 `control/fire=true`。
8. 原子发布 `command.json`，随后通过新状态的 `ipc.last_seq/received/rejected` 诊断是否被接收。

### 11.3 原子文件实现

JSON 读取使用 `openat(O_RDONLY|O_CLOEXEC|O_NOFOLLOW)`，`fstat` 限制为普通文件且长度在 `1..65536` 字节，循环 `read` 处理 `EINTR`，解析失败视为本轮无新值。JSON 写入使用同目录、带客户端 token 的临时文件，循环 `write` 完整写入并关闭，再通过 `renameat` 原子替换 `request.json` 或 `command.json`；异常时仅逐个 `unlinkat` 自己明确命名的临时文件。

图像文件不 mmap 到算法生命周期：读取时完整复制到客户端自有 `cv::Mat`，完成后清空当前请求。这样下一个请求触发仿真器删除上一帧文件时，算法仍持有稳定像素数据。

### 11.4 旧接口收敛

`auto_aim_sim` 直接持有 `RmAimSimClient`，因此真机 `Camera` 和 `CBoard` 不需要仿真多态。删除 Daedalus 分支后：

- `Camera` 只负责海康相机；`CameraBase::ground_truth()` 和 `Camera::ground_truth()` 删除，`SimTargetTruth` 移到新仿真协议命名空间。
- `standard.cpp` 与 `sentry_DGE.cpp` 直接构造 `io::CBoard`，不再通过 `CBoardBase` 选择仿真实现。
- `CBoardBase` 保留为 `McuRosBridge(CBoardBase&)` 和 `mcu_ros_delay_test` 的稳定注入接缝，但注释改为“下位机接口”，不再描述共享内存或仿真实现。
- Foxglove 的 truth 发布参数改为 `rm_aim_sim::SimTruth`，调用只存在于 `auto_aim_sim`；真机入口不再读取或发布仿真真值。

### 11.5 WSL 布局与启动

最终运行采用 WSL 原生目录，避免在 `/mnt/c` 下构建造成的 `p9_client_rpc` 延迟：

```bash
# 终端 1：独立调试器仓库
cd ~/Downloads/sim/rm_aim_sim
./run.sh -- --config=res://configs/vision.cfg --remote

# 终端 2：DGE 分支的 WSL 原生工作副本
cd ~/Downloads/hhh-rm-aim-sim
source /opt/intel/openvino_2024/setupvars.sh
cmake -S . -B build
cmake --build build -j"$(nproc)"
./build/auto_aim_sim configs/rm_aim_sim.yaml
```

## 12. 完成标准

- `codex/rm_aim_sim` 基于 `test`，用户原工作区内容未被覆盖。
- WSL 中两个独立仓库可按文档启动并完成真实像素闭环。
- 真机默认入口行为不依赖调试器，也不包含旧 Daedalus 选择分支。
- 旧 Bevy/Daedalus 活跃接口和冗余代码已逐文件移除，无批量删除操作。
- 新协议、错误处理和坐标/时间转换有自动化测试。
- 构建、测试及端到端验收结果均有可复现命令和明确记录。

