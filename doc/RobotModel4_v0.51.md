# RobotModel4_v0.51 使用说明

源码与命令核对：2026-11-04。

## 1、本次改了什么

- 将在线固定减 1ms 改为 `imu_delay_ms`，支持正、负和小数毫秒；IMU 缓存可重复查询，检查缺样和插值间隔。
- `standard`、`sentry_DGE` 对齐失败时跳过视觉更新，并关闭控制与发射使能；`handeye_test` 跳过该帧投影。
- 增加连续图像、IMU 自动采集，以及棋盘格延时搜索和独立数据复验。原来的 `capture` 仍在启动时打开相机和 IMU，按 s 同时保存图片与四元数。
- 延时求解直接读取采集目录，结果只在终端打印。
- 修复串口重连处理，仿真按相同帧号匹配图像与姿态；测试集中为 3 份，统一由根 `CMakeLists.txt` 构建。

真机延时仍以 **1ms** 为初值，仿真使用 **0ms**。真实延时要通过下面的动态采集测得。

## 2、新增、修改及沿用文件

| 类型 | 文件 | 用途 |
|---|---|---|
| 原有采集 | `calibration/capture.cpp` | 按 SP25 流程启动相机与 IMU，按 s 保存同编号 `.jpg` 和 `.txt`。棋盘格内角点数从 YAML 读取 |
| 原有沿用 | `calibration/calibrate_camera.cpp` | 使用图片计算内参、畸变，终端打印结果 |
| 原有沿用 | `calibration/calibrate_robotworld_handeye.cpp` | 使用图片和四元数计算相机到云台外参；本流程采用此程序 |
| 原有沿用 | `calibration/calibrate_validate.cpp` | 检查固定棋盘格在世界系的位置、姿态波动，终端打印 PASS/FAIL |
| 原有保留 | `calibration/calibrate_handeye.cpp` | 另一种手眼求解入口，本流程无需再运行 |
| 新增 | `calibration/capture_time_sync.cpp` | 自动记录动态图像和独立 IMU 时间序列 |
| 新增 | `calibration/calibrate_time_offset.cpp` | 搜索、复验延时，默认终端输出 |
| 新增 | `tools/imu_time_buffer.hpp/.cpp` | IMU 时间缓存与插值 |
| 新增 | `tools/time_sync_config.hpp/.cpp` | 延时配置读取和查询时间计算 |
| 新增 | `tools/time_sync_recorder.hpp/.cpp` | 原始采集数据写盘、完整性计数 |
| 新增 | `tools/time_sync_session.hpp/.cpp` | 采集配置快照指纹 |
| 新增 | `tools/time_offset_calibration.hpp/.cpp` | 延时搜索、评分与分段一致性检查 |
| 新增 | `io/serial_port.hpp` | Linux 串口收发与关闭 |
| 修改 | `io/cboard.hpp/.cpp`、`io/socketserial.hpp` | 姿态查询接口、串口重连处理 |
| 修改 | `io/camera.hpp/.cpp`、`io/hikrobot/hikrobot.hpp/.cpp` | 限时读图、帧元数据与丢帧统计 |
| 修改 | `io/Daedalus/daedalus_camera.hpp/.cpp`、`io/Daedalus/daedalus_cboard.hpp`、`io/Daedalus/shared.hpp` | 仿真同帧对齐与采集接口 |
| 修改 | `src/standard.cpp`、`src/sentry_DGE.cpp`、`tests/handeye_test.cpp` | 使用配置延时和有效性检查；手眼测试用于在线网格观察 |
| 新增 | `tests/time_sync_core_test.cpp`、`tests/time_sync_capture_test.cpp`、`tests/time_sync_offline_test.py` | 核心、采集和离线流程自动测试 |
| 修改/使用 | 根 `CMakeLists.txt`、`.clang-format`；`configs/calibration.yaml`、`handeye.yaml`、`standard3.yaml`、`sentry.yaml`、`daedalus.yaml` | 统一构建、格式，以及标定和运行参数 |
| 新增文档 | `doc/RobotModel4_v0.51.md`、`doc/IMU与相机时间同步技术文档.md` | 操作流程和算法说明 |

`calibrate_validate.cpp` 在时间同步改动之前就已存在，之前遗漏的是文件清单。这里将完整标定链路列齐，不把原有文件误标为新增。

## 3、标定和延时测试全流程

**按 s 采集图片与四元数 → 内参 → 手眼外参 → 外参验证 → 自动采集两段动态数据 → 终端求延时 → 回填并上车验证。**

以下命令在 **Linux 工程根目录**执行。当前棋盘配置是 **11×8 内角点、60mm 格长**，先确认与实物一致。已有可靠内外参时，核对 `configs/handeye.yaml` 中的参数后可从第④步开始。

### ① 正常编译

```bash
cmake -S . -B build
cmake --build build -j4
mkdir -p data/calibration data/time_sync
```

需要运行软件自检时执行 `ctest --test-dir build --output-on-failure`；其中 Python 离线测试需要 `cv2`、`numpy`、`yaml`。

### ② 用原采集程序保存图片与四元数

核对 `configs/calibration.yaml` 的相机、串口、棋盘尺寸和 IMU 安装旋转 `R_gimbal2imubody`，然后启动：

```bash
./build/capture configs/calibration.yaml -o=data/calibration/static_01
```

程序已同时打开相机和 IMU，**无需额外开启，也无需在内参、手眼步骤间切换 IMU 开关**。

固定底盘与棋盘格，改变云台 Yaw/Pitch，使角点覆盖不同画面位置和倾角。每个姿态停稳后按小写 **s**，保存约 20～30 组；按 **q** 退出。每组为同编号 `1.jpg`、`1.txt` 等，四元数顺序是 **w x y z**。

这批数据可供内参、手眼、外参验证共用，前提是视角和画面覆盖充分。使用新目录采集；原程序每次从编号 1 开始，重复使用旧目录会覆盖同名文件。后续计算按连续编号读取，不要只改图片或四元数中的一个文件。

### ③ 依次计算内参、手眼外参，再验证

先计算内参：

```bash
./build/calibrate_camera data/calibration/static_01 -c=configs/calibration.yaml
```

程序逐图显示角点，**在图像窗口按任意键继续下一张**。结束后，将终端输出的 `camera_matrix`、`distort_coeffs` 写回 `configs/calibration.yaml`。

再用同一批图片和四元数计算手眼外参：

```bash
./build/calibrate_robotworld_handeye data/calibration/static_01 -c=configs/calibration.yaml
```

同样逐图按键继续；将终端输出的 `R_camera2gimbal`、`t_camera2gimbal` 写回配置，平移单位为 **米**。上述两个程序都是读取已有文件计算，不需要再次启动采集或开启 IMU。

检查外参：

```bash
./build/calibrate_validate data/calibration/static_01 -c=configs/calibration.yaml
```

终端打印位置、姿态波动和重投影误差。默认 PASS/FAIL 条件是：至少 5 个有效样本，位置标准差向量的模长不超过 **0.010m**，姿态标准差向量的模长不超过 **0.5°**。重投影误差仅打印，不参与这两个门限。该程序不求延时，也不生成报告文件。

将确认后的 `camera_matrix`、`distort_coeffs`、`R_gimbal2imubody`、`R_camera2gimbal`、`t_camera2gimbal` 同步到 `configs/handeye.yaml` 和实际运行配置（例如 `configs/standard3.yaml`）。保持相机分辨率、曝光、翻转等工作条件一致。各程序不会自动回填配置。

### ④ 自动采集延时数据

按 s 保存的 `.txt` 只有一组匹配后的四元数，没有原始接收时间序列，不能用来重新搜索 delay。延时测试需要下面的连续采集程序，它也会自行打开相机和 IMU。

退出其他占用相机、串口的程序。固定底盘与棋盘格，由操作者转动云台，包含 **Yaw/Pitch 加减速、反向和组合运动**，保持棋盘清晰可见。自动记录期间不按 s，程序不发送云台控制指令。

```bash
./build/capture_time_sync --config configs/standard3.yaml --output data/time_sync/train_01 --duration 45 --image-fps 30
./build/capture_time_sync --config configs/standard3.yaml --output data/time_sync/validate_01 --duration 45 --image-fps 30
```

`--config` 是实际运行配置，必须同时提供相机内参、手眼外参、串口和 `time_sync_board`。本工程的 `configs/standard3.yaml` 已写入标定板几何，因此不再需要第二个配置文件。旧配置若没有 `time_sync_board`，可用 `--board-config configs/calibration.yaml` 单独提供，程序也兼容 `pattern_cols`、`pattern_rows` 和 `square_size_mm|center_distance_mm`。命令行 `--cols/--rows/--square-m` 仍可覆盖文件值。第一段结束后再录第二段，使用不同运动节奏，配置保持一致。`--image-fps 30` 是图像保存率，IMU 独立记录收到的姿态样本，不随图像降采样。输出目录必须尚不存在，重录时更换目录名。

每段生成 `images/`、`images.csv`、`imu.csv`、`session.yaml`、`calibration.yaml`、`events.csv`，供离线求解使用。其中 `calibration.yaml` 是本次采集配置的自动快照。结束后确认终端 `complete=true`；若为 false，查看该段 `events.csv` 和 `session.yaml` 中的原因后重录。记录完整不代表数据已满足延时求解条件。

### ⑤ 计算并复验延时，直接看终端

```bash
./build/calibrate_time_offset
```

终端打印候选延时、当前延时、公共帧数和前后 RMS，重点看：

- `status: candidate`：找到可辨识的候选值。
- `candidate_imu_delay_ms`：候选延时，单位 ms。
- `offline_validation_passed: true`：独立采集数据支持该候选值。
- `validation` 下的 `current_rms_m` 与 `candidate_rms_m`：同一验证数据使用原参数、训练候选值后的误差。

`invalid_data` 表示数据或参数不合格，`inconclusive` 表示无法确定可靠结果，具体看 `reason`。只有候选状态且独立复验通过后，才进入下一步。默认搜索 −20～+20ms、步长 0.1ms，每段至少需要 300 张有效公共图像且跨度不少于 20s；步长不等于实际标定精度。

从工程根目录运行时，程序读取第④步留下的 `data/time_sync/train_01`，若同级有 `validate_01`，会自动复验。只有训练数据时可以计算候选值，`offline_validation_passed` 为 false。自定义采集路径时用 `--session` 和 `--validation` 指定两段数据。结果仅打印到终端。

### ⑥ 回填延时并上车验证

把终端的 `candidate_imu_delay_ms` 填入 `configs/handeye.yaml` 和实际运行 YAML 的 `imu_delay_ms`。其余两个参数先保留现有初值：

```yaml
imu_delay_ms: 1.0  # 用本次终端打印的候选值替换 1.0
imu_max_gap_ms: 20.0
imu_wait_ms: 12.0
```

正延时表示查询更早的 IMU 姿态：`t_query = t_image - imu_delay_ms`。回填到运行配置，保留已经采集的配置快照原样。

先按场地设置 `handeye.yaml` 的 `height`、`grid_num`、`grid_size`；其中网格在世界系 `z=-height` 平面，距离参考云台坐标原点。运行：

```bash
./build/handeye_test -c=configs/handeye.yaml -d
```

观察静止贴合和转动云台时的漂移，按 q 退出后再启动自瞄：

```bash
./build/standard configs/standard3.yaml
```

检查对齐日志中的 `ok` 比例、各类失败次数和 `wait_max_ms`，比较修改前后的运动稳定性。离线程序始终输出 `online_verification: pending_required`，实车效果由这一步确认，不会自动输出上车验收通过。
