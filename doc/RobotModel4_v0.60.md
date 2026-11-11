# RobotModel4_v0.60

版本定位：把 4 板像素 EKF 从“能跑”推进到“观测可解释、失配可恢复、问题可回放”。本版不改变 11 维状态和像素观测的基本框架，重点是投影与雅可比一致、关联可判定、恢复按图像时间分级处理，以及诊断数据可离线复现。

配套阅读：

- [EKF 技术文档](C:/Users/hhh/Desktop/hhh/doc/EKF技术文档.md)：状态、h/H、F/Q/R、关联、NIS、恢复的完整推导与横向对比。
- [时间同步技术文档](C:/Users/hhh/Desktop/hhh/doc/IMU与相机时间同步技术文档.md)：`imu_delay_ms` 与采集标定流程，本版沿用，不重复。

## 1、本版改了什么

| 环节 | v0.51 及以前 | v0.60 |
|---|---|---|
| 投影与雅可比 | h 含畸变，H 用无畸变针孔导数 | h、H 共用同一套 Brown-Conrady 模型，H 补全畸变导数；角度列改为解析式，不再中心差分 |
| 过程噪声 | CWNA / DWNA / DIAG_RATE 三选一 | 只保留 CWNA；DWNA、DIAG_RATE 及旧 `noise_*` 配置键删除 |
| 关联 | anchor + 灯条按横坐标排序传播 | 四板 × 观测 × 左右交换的枚举一一匹配，带漏检/杂波代价和歧义判定 |
| 更新 | 显式求逆 `S^-1`；超界夹紧 | LDLT 求解；Joseph + 对称化 + 正定性检查；越界拒绝整次更新，不夹紧 |
| NIS | 门控后可放大 P 以“重开门” | 只拒观测，不改先验；持续失配交给恢复管理器 |
| 恢复 | 连续 10 帧无接受更新 → 单帧 PnP 重建，速度清零 | 图像时间状态机 + 短窗候选拟合 + 后续帧预测验证后接管；0.5 s 判 LOST |
| 时间 | 收敛用实际时钟 | 预测时间累计 `elapsed_s_`，收敛/预热/失配都按图像时间；重复或倒退时间戳整帧拒绝 |
| 诊断 | 少量字段 | `UpdateReport`、按编号的像素新息、健康状态、拒绝原因、候选状态；新增回放与录制 |
| 调试布局 | 单份布局 | EKF 专用、云台跟踪专用两份，各 5 图 + 状态表 |

## 2、算法要点

### 2.0 统一板位姿与初始化

初始化只使用一条路线：IPPE 候选 → 与 EKF 相同的固定俯仰/零 roll 模型 → 联合优化板位置与 yaw → 统一质量验收 → 整车初值。Solver、Target、恢复模块共用现有 RobustPnp 中的实现；不再通过 `init_prefer_solver_pose` 切换算法，也不再按可见板数量额外翻转初始 yaw。

- 优化使用 EKF 的含畸变投影和解析雅可比，最多 20 次迭代；每次最多 8 次缩步。半径初值仍为 0.2 m，速度和角速度仍为零，单帧不估计这些量。
- 深度必须大于 0.1 m、板朝向相机，四角点 RMS 不超过 `ekf.init_max_rms_px`。当前默认 8 px 是待实测复核的工程起值，不是标定结论。
- 候选位置差小于 2 cm、姿态差小于 2°时视为同一物理结果；其余候选若像素平方误差差值除以 `noise_observation` 小于 2，返回 `ambiguous_pose`，本帧不新建目标。这是工程选解判据，不是严格卡方检验。
- `Solver::solve` 返回 bool，并设置 `pose_valid / pose_status / pose_rms_px`。失败不覆盖位姿；调用者必须检查有效标记。Target 只接收有效结果，并重新核对初值的角点投影。
- 正常跟踪仍使用角点像素；单帧 PnP 失败不阻断合法像素更新。Foxglove 三维检测话题跳过无效位姿，文字状态表显示板位姿失败原因和 RMS。
- 恢复仅复用统一解算产生初值，仍由原窗口拟合及后续帧验证决定接管，不增加第二套恢复算法。

本次仅验证 EKF 构建、Foxglove C++17 语法、9 组无噪声已知板位姿的一致性及无效输入拒绝。真实角点噪声、歧义拒绝率和运行耗时仍需回放及上车检查。

### 2.1 状态与预测

$$
x=[c_x,c_y,c_z,v_x,v_y,v_z,a,\omega,r_f,r_l,h_l]^T
$$

中心匀速、角度匀速：

$$
c_{k+1}=c_k+v_k\Delta t,\qquad a_{k+1}=\operatorname{wrap}(a_k+\omega_k\Delta t)
$$

`predict(dt)` 只接受有限正 dt，否则原样返回；未来查询用副本外推，不推进滤波器时间。

CWNA 过程噪声（q 为加速度谱密度）：

$$
Q_{p,v}=q\begin{bmatrix}\Delta t^3/3 & \Delta t^2/2\\ \Delta t^2/2 & \Delta t\end{bmatrix}
$$

`q_accel_xy=1`、`q_accel_z=1e-4`、`q_alpha=10`；半径、高差没有速度状态，按 $q\Delta t$ 随机游走。

### 2.2 观测 h 与雅可比 H

每根灯条贡献一个 4 维观测：

$$
z_j=[u_t,v_t,u_b,v_b]^T
$$

端点由整车状态生成 3D 点后投影：

$$
h(x)=\pi\!\left(T_{wc}\,p(x)\right),\qquad \pi=\text{针孔}+\text{5 系数畸变}
$$

雅可比按链式拆成四段：

$$
H=\frac{\partial \pi}{\partial n}\cdot\frac{\partial n}{\partial p_c}\cdot\frac{\partial p_c}{\partial p_w}\cdot\frac{\partial p_w}{\partial x}
=D\cdot N\cdot T\cdot B
$$

- `B`：中心三列单位阵；角度列 $z\times(b-c)$；半径列 $\pm u,\pm v$；高差列对侧板置 1。
- `N`：归一化坐标对相机坐标的导数。
- `D`：含 $k_1,k_2,p_1,p_2,k_3$ 的畸变导数乘内参。
- 深度 $\le 0.1$ m 或非有限时整条投影返回空，由上层按投影失败处理。

### 2.3 关联

合法观测要求：large 一致、四角有限且凸、上下端 y 序正确、与已有观测端点距离 ≥ 0.5 px、每帧最多 6 条。

每条候选边先过角点 RMS 门：

$$
e=\sqrt{\tfrac14\textstyle\sum_{k=1}^{4}\|p_k^{obs}-p_k^{pred}\|^2}<T,\qquad
T=\max(40\,\text{px},\,0.5\,w_{pred})
$$

组合代价：

$$
C=30\,n_{\text{miss}}+30\,n_{\text{clutter}}+\tfrac12\left(\mathrm{NIS}+\log\det S+m\log 2\pi\right)
$$

$m=8n$ 为已选边的维数。次优与最优之差小于 `association_ambiguity_gap`（默认 2.0）即标为 ambiguous；本帧仍保留匹配，但可靠性下降、不参与收敛判定。

### 2.4 更新与 NIS

标准更新前 NIS：

$$
\nu=z-h(x^-),\quad S=HP^-H^T+R,\quad \mathrm{NIS}=\nu^TS^{-1}\nu
$$

门控在预热（10 次 correct 且图像时间 ≥ 0.1 s）后启用，拒绝阈值 13.277。被拒观测只计数、不改 P。协方差更新用 Joseph 形式并对称化，再用 LDLT 检查正定；后验越界（半径/高差/速度/角速）整次更新拒绝。

### 2.5 恢复

健康状态：`INIT / HEALTHY / COASTING / DEGRADED / LOST`，全部按图像时间推进。

```text
连续拒绝 ≥3 帧且距上次接受 ≥0.04s
→ 用最近 0.12s 窗口（3~8 帧先验）拟合候选
→ 候选在后续帧预测：RMS<5px、优于主模型 0.7、身份距离<0.8m、未来位置方差<0.04
→ 连续 2 帧且跨度 ≥0.02s 才接管（整体替换 x 与 P）
→ 0.15s 未接管则过期；0.5s 无接受更新或状态越界判 LOST
```

接管后速度、角速度来自窗口拟合，不再“清零重建”。换板优先由关联处理，不触发整车重建。

## 3、配置变化

`ekf:` 段保留的键：`noise_observation`、`q_accel_*`、`q_alpha`、`initial_cov_*`、`anchor_max_*`、`converge_*`、`rotation_speed_limit`、`nis_*`、`init_max_rms_px`、`skip_tracking_pnp`。

已删除、旧 YAML 残留也会被忽略：

```text
process_model
noise_x / noise_y / noise_z
noise_vx / noise_vy / noise_vz
noise_rotation_angle / noise_rotation_speed
auto_reinit_after_no_update
max_no_update_frames
nis_max_consecutive_rejects
nis_reject_cov_inflation
```

新增 `recovery:` 段（`standard3 / sentry / daedalus` 均已配置）：

| 键 | 默认 | 含义 |
|---|---|---|
| `trigger_s` | 0.04 | 距上次接受的触发时间 |
| `window_s` | 0.12 | 候选拟合窗口 |
| `sample_s` | 0.015 | 窗口最小采样间隔 |
| `expiry_s` | 0.15 | 候选有效期 |
| `confirm_s` | 0.02 | 接管所需验证跨度 |
| `coast_s` | 0.15 | 预测仍可标为 reliable 的时长 |
| `lost_s` | 0.5 | 判 LOST 的超时 |
| `max_rms_px` | 5.0 | 候选验证 RMS 上限 |
| `improvement_ratio` | 0.7 | 相对主模型的改善比例 |
| `identity_distance_m` | 0.8 | 候选与主模型中心距离上限 |
| `future_position_variance` | 0.04 | 未来 0.1s 位置方差上限 |
| `max_fit_ms` | 20.0 | 单次拟合时间预算 |
| `min_frames / max_frames` | 3 / 8 | 窗口帧数范围 |
| `max_seeds / iterations` | 4 / 8 | 拟合初值与迭代上限 |

`Tracker` 另读 `max_frame_gap_s`（缺省 0.1 s）：图像时间间隔超限直接中断并回到 lost。

## 4、新增与修改文件

| 类型 | 文件 | 作用 |
|---|---|---|
| 新增 | `tasks/auto_aim/rmcs_ekf/projection.{hpp,cpp}` | 含畸变投影、解析 H、相机合法性检查 |
| 新增 | `tasks/auto_aim/rmcs_ekf/association.{hpp,cpp}` | 观测合法性、枚举关联、代价与歧义 |
| 新增 | `tasks/auto_aim/rmcs_ekf/recovery.{hpp,cpp}` | 健康状态机、窗口候选、接管判定 |
| 新增 | `tools/ekf_record.hpp` | 原始观测与跟踪结果录制（JSONL） |
| 新增 | `tools/ekf_replay.cpp` | 固定输入回放真实 Tracker |
| 新增 | `foxglove_layouts/ekf_debug_layout.json`、`gimbal_tracking_layout.json` | 两份调试布局 |
| 修改 | `tasks/auto_aim/rmcs_ekf/rmcs_ekf.{hpp,cpp}` | 单一路线 Q、LDLT/Joseph、诊断与状态边界 |
| 修改 | `tasks/auto_aim/target.{hpp,cpp}`、`tracker.{hpp,cpp}` | 接入 RecoveryManager、健康状态与录制 |
| 修改 | `CMakeLists.txt`、`tasks/auto_aim/CMakeLists.txt` | `EKF_ONLY` 构建、新源文件 |
| 修改 | `tools/Foxglove/foxglove.cpp` | 健康、候选、残差、NIS、状态表字段 |
| 修改 | `configs/{standard3,sentry,daedalus}.yaml` | 清理旧键、增加 `recovery` |

## 5、编译与运行

完整工程（Linux 小电脑，需要 OpenVINO / Ceres / 相机 SDK）：

```bash
cmake -S . -B build
cmake --build build -j4
```

只编译 EKF 与回放（只需 OpenCV、Eigen、yaml-cpp、fmt、spdlog、nlohmann_json，不需要相机和模型）：

```bash
cmake -S . -B build-ekf -DEKF_ONLY=ON
cmake --build build-ekf -j4
```

回放一段录制：

```bash
./build/standard configs/sentry.yaml        # 运行时自动录制
./build-ekf/ekf_replay logs/ekf/<时间戳> trace.jsonl
```

录制默认写入 `logs/ekf/<时间戳>/`，可用 `HHH_EKF_RECORD_DIR` 改基目录。目录内：

| 文件 | 内容 |
|---|---|
| `observations.jsonl` | 每帧原始角点、颜色/类型、相机内外参、可选 omni 队列 |
| `trace.jsonl` | 跟踪状态、11 维状态、完整 P、残差、NIS、健康与恢复事件 |
| `manifest.json` | schema、源码指纹、配置指纹、OpenCV 版本 |
| `config.yaml` | 该次运行的配置快照 |

回放不依赖 Foxglove、相机或仿真器，用于同输入对比 v0.51 与 v0.60。

构建状态：`EKF_ONLY` 配置、编译及 `ekf_replay --help` 已在 WSL（Ubuntu 22.04，OpenCV 4.5.4）验证通过；完整工程、仿真与实车尚未在本版验收。

## 6、调试界面

两份布局均保留“窗口”画面和原 3D 面板，右侧 5 条曲线，EKF 布局另有中文状态表。

- [EKF 调试布局](C:/Users/hhh/Desktop/hhh/foxglove_layouts/ekf_debug_layout.json)：水平残差、垂直残差、更新前 NIS、半径与高度差、位置标准差，加状态表（健康、reliable、候选、接受/拒绝、关联编号、恢复次数）。
- [云台跟踪布局](C:/Users/hhh/Desktop/hhh/foxglove_layouts/gimbal_tracking_layout.json)：Yaw 指令/反馈、Pitch 指令/反馈、两轴误差、目标转速、有效状态与开火指令。

判读边界：

- 残差是“观测 − 更新前预测”，单位 px；分别记录上下端点的 u/v，按模型编号 0–7 发布，无观测时为 null。
- NIS 是单条四维统计量，参考线 9.488、门控线 13.277；曲线越界不等于观测错误，还可能是机动、错关联或噪声模型偏小。
- `reliable` 只表示“可以信任当前预测”，不等于真实误差小；实车没有真值，精度结论需要打靶或录像复核。
- 曲线重合只说明云台跟上了指令，是否真正命中还要看画面与弹道。

## 7、验收方案

### 7.1 离线回放（先做，成本最低）

同一段 `observations.jsonl`，分别用 v0.51 和 v0.60 回放，比较：

| 指标 | 期望 |
|---|---|
| 关联歧义帧比例 | v0.60 能标出并减少错关联 |
| 更新前 NIS 分布 | 不因门控变化整体失真；拒绝集中在尖峰 |
| 预测误差（用后续帧观测当准真值） | 恢复后明显下降，稳态不劣化 |
| 错误接管次数 | 0；允许“不接管” |
| 恢复耗时 | 从持续拒绝到 reliable 的时间不超过 0.2~0.3 s |
| 单帧耗时 | association + update 相对 v0.51 增幅可接受 |

回放验证的是“同一输入下算法行为”，不能替代真实精度。

### 7.2 Linux 仿真

用 Daedalus 跑遮挡、换板、急停反转、小陀螺和单板弱观测场景，检查：

- 遮挡时进入 COASTING、P 增长、恢复后回 HEALTHY；
- 错误关联时 ambiguous 置位、不接管错误候选；
- 长时间无观测最终 LOST，而不是永久保留；
- 无发散、无 NaN、无 P 非正定。

### 7.3 实车

按“静止 → 缓慢平移 → 小陀螺 → 急停/反转 → 遮挡”顺序录制，每段保留 `logs/ekf/<id>` 与配置。看窗口与两份布局，重点确认：

- 残差无持续单边偏置（若有，先查内参/外参/时间延迟，不要先调 Q/R）；
- 换板时残差短时跳变后收敛，不触发整车重建；
- 急停反转后 0.2 s 内 reliable 恢复或明确进入候选验证；
- 开火/控制行为与 v0.51 一致（本版未改控制策略）。

### 7.4 通过标准

1. 完整工程与 `EKF_ONLY` 均编译通过。
2. 回放中无崩溃、无 NaN、协方差始终正定。
3. 仿真与实车均未出现错误接管；恢复后预测误差回到该场景稳态水平。
4. 与原版相比，基础跟踪（静止/匀速）不劣化；机动与遮挡场景有可量化改善。
5. 若某项未达标，保留数据并说明原因，不以“曲线好看”代替指标。

## 8、已知边界

- 单板近正对时深度、朝向与半径弱可观测，任何关联或恢复策略都无法凭空补信息；残差小不代表位置准。
- `R=40I` 仍是配置例，未按检测质量/距离/灯条长度调度；本版不解决噪声标定问题。
- 状态越界采用“拒绝更新”，可能整帧丢弃；边界值需按实车数据调整，而不是长期依赖夹紧。
- 候选恢复依赖窗口内图像时间连续；丢帧或时间跳变会直接失效。
- 录制默认开启，长期运行需关注磁盘占用；目前没有 YAML 开关，只能改代码或调整 `HHH_EKF_RECORD_DIR` 基目录。
- Daedalus 无真值精度结论；v0.51 的 `imu_delay_ms` 仍需独立标定，二者不要混为一次验收。

## 9、后续

- 用实车数据标定 Q/R 与状态边界，再评估是否需要自适应噪声。
- LED 点阵装甲迁移时，只需替换“固定物理角点”的定义与尺寸，投影/H/关联/恢复结构可沿用；点序、遮挡角点可信度和尺寸必须先核对。
- 若实测显示恢复仍偏慢，再考虑多假设或短时外推，而不是回到“固定帧数重建”。
