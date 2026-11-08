# IMU与相机时间同步技术文档  
编写与源码核对日期：2026-11-04 

## 0、统一符号与时间定义

### 0.1、必须区分的时刻

| 符号/字段 | 定义 | 当前是否能直接获得 |
|---|---|---|
| $t_e$ | 图像所代表的曝光时刻；短曝光、近似对称成像时常取曝光中点 | hhh 现有上层接口不能直接给出 |
| $t_s$ | IMU 姿态对应的物理时刻 | 当前串口包没有该时间戳 |
| $t_C$ | 图像在主机打点的时刻 | GetImageBuffer 成功返回后记录 |
| $t_I$ | 姿态包在主机接收回调打点的时刻 | 当前可以记录 |
| $t_q$ | 本帧用于查询姿态缓存的时间 | $t_C-\Delta$ |
| $\Delta$ | 图像—IMU 有效相对时间偏移 | 由本次工具自动标定，实车值待采集 |
| $t_a,t_b$ | 包围 $t_q$ 的两条 IMU 样本时间 | 从完整接收序列查找 |
| $q_a,q_b$ | 两条样本的姿态四元数 | qw/qx/qy/qz |
| $h=t_b-t_a$ | 插值间隔 | 应检查上限，不能只检查 $h>0$ |

硬件已知条件为：下位机 IMU 采样/解算 1kHz，上发姿态 100Hz，相机海康 MV-CS016-10UC。1kHz 解算周期不等于相机—姿态相对延迟 1ms；100Hz 上发也不意味着偏移只能按 10ms 的整数倍估计。

采集与在线查询使用同一进程的 steady_clock。两个独立程序各自建立的相对零点不得直接混合；模拟时间、ROS 时间、设备 tick、主机单调时间也不得在未映射时相减。

### 0.2、delay 的物理含义与符号

以主机单调时钟表示物理时间，近似写成：

$$
t_C=t_e+d_C,\qquad t_I=t_s+d_I.
$$

在姿态序列中找到代表 $t_e$ 的样本，需要查询 $t_e+d_I$。因此：

$$
t_q=t_C-\Delta,\qquad \Delta=d_C-d_I.
$$

- $\Delta>0$：查询更早的 IMU 接收时间。
- $\Delta<0$：查询更晚的 IMU 接收时间，在线可能需要等候后续样本。
- $d_C,d_I$ 包含各自从数据代表时刻到打点位置的延迟；相机链路可能包含曝光、读出、传输和调度，姿态链路可能包含解算、上发排队、串口传输和接收调度。

只有固定或近似固定的相对偏移能由一个常量很好补偿。姿态滤波的频率相关相位滞后、通信随机抖动、丢包和时间漂移不能全部压缩成一个可靠的 delay。

同样的数据，若给全部 IMU 时间戳加 $d$ 后按 $t_C$ 查询，等价于在原序列按 $t_C-d$ 查询。这解释了 ROS 工程的正 timestamp_offset 与 SP25 回退查询之间的关系。

### 0.3、曝光中点与主机接收时间

曝光中点满足 $t_{\mathrm{mid}}=t_{\mathrm{start}}+T_{\mathrm{exp}}/2$，前提是已知曝光开始时刻及实际曝光长度。将“取图返回时刻减半曝光”只扣除了其中一项，并没有自动扣除读出、USB 传输和主机调度时间。

v0.51 保留 hhh 现有图像打点语义，用标定值吸收稳定的相对偏差。若未来改成曝光中点时间、调整打点位置或接入相机设备时间，应建立新数据版本并重新标定；不能沿用旧 delay 再叠加半曝光补偿。

LED 点阵装甲的检测方式不改变这套时间定义。动态发光、曝光期间的运动模糊可能影响关键点观测，当前延迟标定仍使用固定、清晰的几何标定板，不把 LED 显示变化作为时钟基准。

### 0.4、SLERP 插值与有效性

先检查四元数有限且范数有效，再归一化。如果 $q_a\cdot q_b<0$，令 $q_b\leftarrow-q_b$，选择同一旋转的最短表示。对严格包围区间：

$$
\lambda=\frac{t_q-t_a}{t_b-t_a},\quad 0<\lambda<1,
$$

$$
q(t_q)=
\frac{\sin[(1-\lambda)\theta]}{\sin\theta}q_a+
\frac{\sin(\lambda\theta)}{\sin\theta}q_b,\qquad
\theta=\arccos(\operatorname{clamp}(q_a\cdot q_b,-1,1)).
$$

小角度使用稳定的近线性实现，最后归一化。精确命中有效样本直接返回该样本，不能错误地要求一定存在左右两条额外样本。

初始最大插值间隔设为 20ms，供 100Hz 链路验证。超过间隔、早于缓存、缺少右样本、四元数无效分别返回明确状态。在线允许最多 12ms 等候右样本，离线不等待、不外推。数值仅为初始工程设置，需按实测等待和缺样统计调整。

将越界 $\lambda$ 强行截到 0 或 1，会把越界伪装成成功。返回 identity 或最新姿态而不报告降级，也会把时间错误带入观测变换。

## 1、hhh 当前实际流程

### 1.1、相机与 IMU 打点

~~~text
海康曝光与传输
→ MV_CC_GetImageBuffer 返回
→ steady_clock::now() 记录 t_C
→ 图像转换、队列
→ camera.read 返回原时间

下位机 IMU 1kHz 解算
→ 100Hz 姿态上发
→ 接收并识别完整串口帧
→ CBoard::callback 记录 t_I
→ 解码四元数、入姿态队列、通知观察者
~~~

[相机打点](C:/Users/hhh/Desktop/hhh/io/hikrobot/hikrobot.cpp) 在颜色转换之前；[IMU 打点](C:/Users/hhh/Desktop/hhh/io/cboard.cpp) 是接收处理时间。[当前 IMU 包](C:/Users/hhh/Desktop/hhh/io/cboard.hpp) 仅含帧头、四个 int16 四元数分量、帧尾，没有源采样时刻与 MCU 序号。

### 1.2、实施前的入口差异与本版统一结果

| 入口 | 实施前偏移 |
|---|---|
| standard | 固定 $t_C-1ms$ |
| sentry_DGE | double imu_delay_ms，缺省 1ms |
| handeye_test | 整数 delay，按毫秒回退 |
| calibration/capture | 按图像时间查询，保存图片及一组已匹配姿态 |
| 若干未启用的旧 sentry 入口 | 仍保留固定 1ms |

[standard](C:/Users/hhh/Desktop/hhh/src/standard.cpp)、[sentry_DGE](C:/Users/hhh/Desktop/hhh/src/sentry_DGE.cpp)、[handeye_test](C:/Users/hhh/Desktop/hhh/tests/handeye_test.cpp)、[capture](C:/Users/hhh/Desktop/hhh/calibration/capture.cpp)。

实施前的 imu_at 消费队列，缺少完整时间范围检查，不适合多轮离线 delay 搜索。以上历史行为对应基线提交 `5852b846955abbe08eff7b91f70c6f2595369d2b`。

v0.51 已将启用的 standard、sentry_DGE、handeye_test 统一到 `double imu_delay_ms` 和有状态的非消耗式 `imu_query`。实机配置初值 1ms；仿真同帧配对为 0ms。采集工具独立保存原始图像时间与 IMU 接收序列；旧 capture/Recorder 的每图匹配姿态不能替代这一数据集。当前实现见 [姿态查询](C:/Users/hhh/Desktop/hhh/tools/imu_time_buffer.cpp)、[参数解析](C:/Users/hhh/Desktop/hhh/tools/time_sync_config.cpp) 和 [采集入口](C:/Users/hhh/Desktop/hhh/calibration/capture_time_sync.cpp)。

### 1.3、原有标定采集与延时采集的分工

`calibration/capture.cpp` 启动时已打开相机与 IMU，按小写 s 同时保存同编号图片和四元数，按 q 退出。当前配置默认采集 IMU，无需在内参和手眼步骤间重复开启或切换开关。四元数 `.txt` 顺序为 w、x、y、z，没有原始接收时间序列。

固定底盘和棋盘格，每次改变云台姿态后停稳保存。视角覆盖充分时，同一批数据依次用于 `calibrate_camera`、`calibrate_robotworld_handeye` 和 `calibrate_validate`。前两个程序逐图显示，需按键继续；结果在终端打印，由操作者写回配置。`calibrate_validate.cpp` 是原有外参验证程序，输出位置、姿态波动和 PASS/FAIL，不搜索延时。

空间标定通过后，另用 `capture_time_sync` 记录动态过程。它自行打开两路设备，自动保存图像时间戳和独立 IMU 接收序列，不需要按 s。增加这一步是因为延时搜索需要完整时间序列，不是因为原采集没有打开 IMU。两种采集程序先后运行，退出前一个后再启动下一个。

完整文件清单和唯一一套操作命令见 [使用说明](RobotModel4_v0.51.md)。

### 1.4、现有录像为何不够

Recorder 的每帧记录为“时间 + 一组四元数”，默认按 30Hz 保存图像。这里的 q 通常已经按当前 delay 匹配过，缺少独立的原始 100Hz 姿态时间序列。改变 delay 后无法重新查询真实的前后样本，因此旧录像不能自动充当完整时间标定数据。[录像器](C:/Users/hhh/Desktop/hhh/tools/recorder.cpp)。

已有 ImuSample 和 set_imu_observer 可以取得每条通过解析校验的原始姿态及接收时间。独立采集器可复用该接口。观察者回调必须轻量，不能在串口线程内编码图像、写盘或求解 PnP。[接口](C:/Users/hhh/Desktop/hhh/io/cboard.hpp)、[回调](C:/Users/hhh/Desktop/hhh/io/cboard.cpp)。

现有观察接口是单槽，ROS 桥也使用它。采集与桥接共存时，需要统一分发，不能后注册者覆盖先注册者，也不能把只保留最新值的 mailbox 当作无丢失原始 IMU 日志。

## 2、SP25：配置偏移与投影验证

### 2.1、主程序对齐

SP25 主程序使用 $q=\operatorname{imu\_at}(t_C-1ms)$。相机在取图返回后打主机时间；CAN 姿态在接收回调打主机时间，并在队列中进行四元数插值。[主入口](<D:/1RM_vision/sp25/同济-2025/src/standard.cpp:59>)、[插值与接收](<D:/1RM_vision/sp25/同济-2025/io/cboard.cpp:22>)。

这是相对软件补偿，并非由 1kHz 采样周期直接推导出的硬件同步关系。

### 2.2、handeye_test 的实际能力

handeye_test 读取 height、grid_num、grid_size、整数 delay，生成固定世界网格，再用补偿后的姿态和已有外参投影到画面：

~~~text
世界网格点
→ 图像时刻减指定 delay 查询姿态
→ world2pixel
→ 叠加白色网格点
→ 人工观察静止与运动时的贴合情况
~~~

它不检测真实标定板、不定义自动评分、不遍历 delay，也不输出最佳偏移。[测试源码](<D:/1RM_vision/sp25/同济-2025/tests/handeye_test.cpp:37>)。

静止时也偏离，先检查空间标定和网格定义；只有运动时随速度、方向产生变化，才进一步排查时间对齐。这是诊断线索，不是仅凭视觉现象就能唯一识别误差来源。

### 2.3、手眼外参与时间标定的区别

calibrate_handeye 使用图片及对应姿态求空间外参；calibrate_robotworld_handeye 还估计标定板与世界的空间关系。这两份标定程序没有搜索 delay。空间外参应该先在多个稳定姿态下核查，再用动态数据标定相对时间。[手眼求解](<D:/1RM_vision/sp25/同济-2025/calibration/calibrate_handeye.cpp:153>)、[联合空间求解](<D:/1RM_vision/sp25/同济-2025/calibration/calibrate_robotworld_handeye.cpp:167>)。

## 3、武科：固定标定板的自动延迟搜索

### 3.1、采集与测量 Z

delay_test 检测棋盘格角点，亚像素细化后用 IPPE PnP 求出板原点位置 $\mathbf z_i=\mathbf p_i^C$，保留图像时间；另存云台姿态对应的时间与相机到世界变换。

源码硬编码 11×8 个内角点、0.15m 格长。坐标点生成的平面为 x=0，仍属于 IPPE 要求的共面点；hhh 移植时只需保持模型、PnP 输出和后续坐标变换约定一致，不能只复制尺寸或默认单位。[测量部分](<D:/1RM_vision/武科/awakening-main/src/runtime/test/delay_test.cpp:123>)。

### 3.2、评分与搜索

武科搜索量记作 $\delta$，查询方式为 $t_i+\delta$：

$$
\mathbf p_i^W(\delta)=T_{WC}(t_i+\delta)\mathbf p_i^C.
$$

将结果取三维，计算均值及位置 RMS：

$$
J(\delta)=\sqrt{\frac1N\sum_i
\|\mathbf p_i^W(\delta)-\bar{\mathbf p}^W(\delta)\|^2}.
$$

搜索 −10～+10ms，步长 100μs，选择最小值。它使用标定板在世界系静止这一约束，不要求事先精确测量标定板的世界位置。[评分与搜索](<D:/1RM_vision/武科/awakening-main/src/runtime/test/delay_test.cpp:332>)。

它的离线 search_pose 对范围外查询返回无效，区间内平移线性插值、旋转 SLERP。不要与该工程在线 TF 缓存允许向未来外推的行为混淆。[离线插值](<D:/1RM_vision/武科/awakening-main/src/runtime/test/delay_test.cpp:296>)。

### 3.3、现有补偿的累计关系

该测试先把姿态时间设置为 $t_I-c$，其中 c 来自 serial_send_to_image_microseconds，然后查询 $t_C+\delta$。相当于在原始 IMU 接收时间序列查询 $t_C+c+\delta$。

因此仍按武科约定部署时：

$$
c_{\mathrm{new}}=c_{\mathrm{old}}+\delta^\*.
$$

hhh 若使用相同原始打点，则 $\Delta=-(c+\delta^\*)$。但武科的图像时间还减去了半曝光，hhh 当前没有这一步，因此跨工程不能只套这个符号关系搬运数值，必须在 hhh 自身链路重新标定。[既有补偿](<D:/1RM_vision/武科/awakening-main/src/runtime/test/delay_test.cpp:183>)、[相机半曝光回退](<D:/1RM_vision/武科/awakening-main/src/utils/drivers/hik_camera.cpp:87>)。

### 3.4、实际运行条件与移植修正

原程序主动发送正弦云台指令，Yaw 幅值 30°、Pitch 幅值 5°，均为 1Hz；发送任务设为 1000Hz。这个 1000Hz 是程序任务频率，不证明姿态接收频率为 1000Hz。hhh 首版只借鉴采集与求解过程，云台动作由操作者安排。[指令生成](<D:/1RM_vision/武科/awakening-main/src/runtime/test/delay_test.cpp:221>)。

需要补齐的有效性判断：

1. 不同候选会因查询越界而使用不同帧，可能造成评分偏差；hhh 固定公共帧集合。
2. 单帧 PnP 成功不代表解可靠；需重投影、深度、角点顺序及模糊检查。
3. 两帧可算出 RMS，不足以证明可辨识；需要足够动作与独立验证。
4. 原程序无有效候选时可保留 double 最大有限值，但最终只检查 isfinite，不能可靠阻止输出无效 Best delay；hhh 必须使用显式候选有效计数和结果状态。
5. 最小值本身不能说明精度；hhh 计算完整扫描曲线与分段结果后再判定状态。终端显示求解结果。

以上是本地源码静态核对，不是对原队实车效果的结论。

## 4、南理、FYT、中石大的相关做法

### 4.1、你提供的南理版本

该目录 Git 远端为 ssh-hhh/rmcs_auto_aim，因此以下只描述你提供的本地版本，不将全部实现归为原队官方发布。

源码配置海康 Line2 的 ExposureStartActive 输出，接收板端曝光事件时间，将相机设备时间与 BoardClock 时间配对，建立：

$$
t_B\approx a\,t_{\mathrm{camera-device}}+b.
$$

初始最小二乘拟合，后续 RLS 跟踪关系，检测帧号间断和时钟关系残差。IMU 在板端时间域进行插值；缺少可用快照时不输出有效对齐帧。[曝光配置](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/io/capturer/hikcamera.hpp:369>)、[同步模型](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/io/capturer/linear_sync_model.hpp>)。

移植前有两处需核对：名为 half_exposure_time_ 的量按完整 exposure_us 初始化；如果曝光事件代表开始时刻，查询目标就不是预期中点。另一个输出 exposure_timestamp 被赋为曝光事件到达主机的时间，不能据此宣称后续 EKF 的时间也已完全转换到真实曝光时刻。[初始化](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/io/capturer/capturer.cpp:45>)、[查询及输出时间](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/io/capturer/capturer.cpp:470>)。

板端固件的实际边沿捕获和安装接线没有在本次核验，不宣称已经验证你的具体相机节点与电气接入条件。

### 4.2、FYT2024 与中国石油大学

FYT 相机在回调颜色转换后用 ROS now 打点；串口姿态按 now+timestamp_offset 发布 TF，配置为 +6ms，再按图像时间做坐标查询。[相机](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_hardware_driver/rm_camera_driver/src/daheng_camera.cpp:270>)、[串口与 TF](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_hardware_driver/rm_serial_driver/src/serial_driver_node.cpp:98>)。

中石大在检测节点取图后打时间，按该时间查询 TF。bringup 的 timestamp_offset 为 +4.3ms，独立串口配置为 +6ms，实际取决于启动所加载的文件。[图像时间与查询](<D:/1RM_vision/中国石油大学/src/rm_auto_aim/armor_detector/src/detector_node.cpp:453>)、[bringup 配置](<D:/1RM_vision/中国石油大学/src/rm_vision_bringup/config/node_params.yaml:7>)。

这些软件偏移的数值与打点位置、传输链路有关；不能因为采用 ROS TF 就认为已经获得硬件曝光与 IMU 采样时间同步。

## 5、hhh v0.51 落地算法

### 5.1、设计待估参数、观测 Z 与变换函数 h

这是独立标定问题，待估参数为相对偏移 $\Delta$ 和固定标定板参考点 $\boldsymbol\mu^W$。实现将内角点网格中心作为标定板坐标原点。它不占用 RobotModel4 的目标状态向量。

观测为：

$$
Z=\{(t_{C,i},\mathbf p_i^C)\}_{i=1}^N,\qquad
U=\{(t_{I,j},q_j)\}_{j=1}^M.
$$

令 $s_i=t_{C,i}-\Delta$，并定义：

$$
h_i(\Delta)=R_{WG}(s_i)
(R_{GC}\mathbf p_i^C+\mathbf t_{GC})
+\mathbf t_{WG}(s_i).
$$

$R_{GC},\mathbf t_{GC}$ 是相机 OpenCV 坐标到云台坐标的外参。固定底盘实验中，云台参考原点在世界中应固定；若机械轴系使参考原点随 Pitch 变化，则必须使用正确轴系几何，不能把其平移误差解释为延迟。

hhh 的四元数还经过 R_gimbal2imubody 坐标变换，不能直接把原始 q 当成 $R_{WG}$。标定程序必须复用与在线相同的坐标约定。[Solver 姿态变换](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/solver.cpp)。

### 5.2、目标函数与时间敏感性

$$
\boldsymbol\mu^\*(\Delta)=\frac1N\sum_i h_i(\Delta),
$$

$$
\Delta^\*=\arg\min_{\Delta\in[\Delta_{\min},\Delta_{\max}]}
\sqrt{\frac1N\sum_i
\|h_i(\Delta)-\boldsymbol\mu^\*(\Delta)\|^2}.
$$

一次会话内保持板位置、底盘基座和姿态参考系不变。发生移动、IMU 重置或设备重连时分成不同会话，不把不同参考系的数据合并求一个均值。

若用世界角速度 $\omega_W$ 和云台原点速度 $v_{WG}$ 表示时间敏感性：

$$
\frac{\partial h_i}{\partial\Delta}
=
-\omega_W(s_i)\times[h_i(\Delta)-\mathbf t_{WG}(s_i)]
-v_{WG}(s_i).
$$

去均值残差的导数还应减去所有帧导数的均值。该式用于解释可辨识性和误差规模，本版搜索不依赖解析雅可比，也不新增在线时间 EKF。

在固定轴匀速的理想模型中，时间错位可能仅造成固定空间变换，位置离散评分无法区分。应采集变化角速度与不同方向的动作。已知空间外参也不能自动消除这种评分的退化。

### 5.3、统一搜索与数据质量

初始搜索区间为 [−20,+20]ms，步长 0.1ms；400 个间隔、401 个候选。参数为启动设置，最小值位于边界时标为 inconclusive，依据数据重新设置范围后重跑整条曲线。

处理顺序固定为：

1. 校验元数据、时间顺序、四元数、图片对应关系。
2. 在图像上检测角点、亚像素细化，求 PnP 并检查正深度、重投影误差；参考点取内角点网格中心。
3. 质量筛选只依赖图像和固定的几何规则，不依赖候选 delay 或目标 EKF 的输出。
4. 对每个候选检查插值可用性，取全部候选均可用的图像交集 S。
5. 在同一个 S 上计算全部候选的 RMS；没有足够公共样本则判为数据不足。
6. 计算完整曲线和各连续时间块的最优值，在独立采集会话复验；在终端打印结果。

初始采集质量设置：每个用于求解或复验的会话至少 300 张有效图像，覆盖至少 20s 的动态过程；PnP 角点重投影 RMS 初始上限为 1.5px。它们只是起步门槛，须记录并根据静止噪声和图像质量审查，达到门槛也不保证可辨识。低质量数据不能通过不断放宽门槛获得“合格”结果。

对称棋盘格保持稳定的可见朝向。实现通过中心原点避免整体角点顺序翻转改变参考点；IPPE 候选解检查正深度并按重投影 RMS 选择，选择不依赖待标定的 delay。目前没有额外的时序 PnP 分支跟踪器，仍需结合残差、分段一致性和独立复验识别几何不稳定数据。

### 5.4、结果状态与参数应用

候选值字段是 `candidate_imu_delay_ms`。当前程序只有以下三个状态：

| 状态 | 含义 |
|---|---|
| invalid_data | 数据损坏、元数据冲突、时间不合法或公共样本不足 |
| inconclusive | 运动不足、曲线平坦、多处相近极小值、最小值在边界、分段结果不一致 |
| candidate | 扫描得到可辨识候选；独立复验情况另看 `offline_validation_passed` |

只有 `status: candidate` 且 `offline_validation_passed: true` 才进入上车验证。未提供独立会话时该布尔值为 false；独立复验不支持训练候选时，状态转为 inconclusive。即使状态不是 candidate，也可能打印扫描最小值，不能只取数字回填。

离线程序始终保留 `online_verification: pending_required`，不会输出 validated。在线等待时间、失败分布和实际画面由操作者验收。

默认在终端打印状态、原因、候选值、当前配置值、公共帧数及前后 RMS。验证集的 `candidate_rms_m` 使用训练候选值计算。基线来自采集配置的当前值，缺省 1ms。

无标定数据时不能预先写一个“最优 delay”。工具不会自动覆盖运行 YAML；通过复验后人工回填运行配置，不修改采集快照。扫描步长不作为测量精度；固定偏移消除的也不是随机抖动。

统一配置名为 double imu_delay_ms。旧 handeye 的 delay 在新键缺失时作为兼容输入并告警；两键冲突则配置失败。正负号、单位在结果、配置、日志和查询函数中保持一致。

### 5.5、在线边界与 EKF 的关系

在线查询返回四元数、状态、包围时间、插值比例、实际等待时间和最大间隔检查结果。失败帧跳过视觉更新并关闭控制与发射使能，记录原因；Tracker/EKF 的预测时间继续使用下一次有效图像的原始时间。

12ms 等待预算是端到端延迟的一部分，必须测量。负 delay 可能使查询落在未来，即使离线评分最优也未必能在该预算内取得右样本，需要在实机验证覆盖率。

本版仍以原始图像时间 $t_C$ 推进 Tracker/EKF；$t_C-\Delta$ 只是姿态接收序列的查询坐标，不能把它误当成已还原的曝光时间。

相机绝对延迟、执行延迟和弹道飞行时间仍各有物理含义。它们不能与本次相对时间偏移直接相加后当作统一预测提前量。

时间不确定度在图像中会形成与运动有关的误差，通常不是独立白噪声。本版不自动把 delay 或接收间隔当作 R 的方差，也不调整目标模型的 Q/P；先完成对齐和诊断，再评估残余误差。

## 6、自动采集方法与数据格式

### 6.1、会话文件与字段

所有相对时间都减去同一个会话 t0，保存为 int64 纳秒，计算时按需换算成秒或毫秒。文件名序号不替代时间戳。以下是连续采集的原始数据，不是求解后的报告：

| 文件 | 必需内容 |
|---|---|
| session.yaml | schema_version=1，session_id，clock=steady_clock，timestamp_semantics，camera/IMU 类型、曝光、图像保存率、源码 HEAD、配置校验摘要、记录计数和异常计数 |
| images/ | 按保存率记录的原始图像 PNG |
| images.csv | frame_id、t_image_ns、image_path、可选 camera_frame_id、device_timestamp_ticks、exposure_us |
| imu.csv | sample_id、t_imu_receive_ns、qw、qx、qy、qz、valid |
| calibration.yaml | K、畸变、R_camera2gimbal、t_camera2gimbal、R_gimbal2imubody、棋盘格内角点行列与格长、长度单位 |
| events.csv | t_event_ns、event_type、关联本地序号、原因 |

会话中的 `calibration.yaml` 是采集程序自动保存的配置快照，来源是 `--config` 指定的文件；正常流程使用 `configs/handeye.yaml`。无需另建或手改此快照。

在工程根目录运行 `./build/calibrate_time_offset` 会读取 `data/time_sync/train_01`，若有 `data/time_sync/validate_01` 则同时复验。结果仅打印到终端。

不可获取的设备时间戳或帧号写空值，不填 0 冒充真实数据。设备 tick 未知频率、起点或事件语义时仅留作诊断，不直接用于融合。

frame_id、sample_id 是 PC 本地记录序号，不代表 MCU 的采样序号；现有协议不能据此准确统计下位机每一条丢失的原始 IMU 样本。

统一使用四元数顺序 qw/qx/qy/qz。Eigen coeffs() 的内部顺序为 x/y/z/w，序列化必须显式对应，不能直接把内存顺序写成 wxyz。

### 6.2、线程与缓存

~~~text
相机采集线程 ── 原始图像、原打点、元数据 ──→ 有界图像写入队列
                                                    ↓
                                                 images/

串口接收线程 ── ImuSample 的轻量拷贝 ──→ 有界 IMU 日志队列
                                                    ↓
                                                  imu.csv

两路记录完成 → 离线检测与时间搜索 → 终端摘要
~~~

图像队列拥有独立图像数据，不能保存指向已归还 SDK 缓冲区的悬空引用。记录线程按条目自身时间写盘，不以写盘时刻替代采集时间。

可在完成打点后按明确规则抽样保存图像，例如相机保持实际工作采集率、日志保存 30Hz 图像；IMU 必须保持原始 100Hz 接收序列，不随图像抽样。保存率和源帧号需记录，不能用视频的恒定 FPS 推算实际帧时刻。

使用有界队列并记录溢出、丢弃数。正常停止先停止生产，再排空已接受的数据，最后写计数摘要并关闭文件。硬件停止或异常退出的截断会话标明 incomplete，不能静默当完整数据使用。

### 6.3、操作步骤

1. **完成空间标定。** 用原 `capture` 在不同稳定姿态下保存图片与四元数，依次计算内参、手眼并运行 `calibrate_validate`。确认后将参数写入自动采集和实际运行配置。
2. **自动记录两段动态数据。** 固定底盘与棋盘格，用 `capture_time_sync` 录制训练、独立复验两个会话，各约 30～60s，使用不同节奏的 Yaw/Pitch 加减速、反向和组合动作。保持曝光等配置相同，图像清晰，设备由程序自行打开。
3. **停止与检查。** 确认 `complete=true`。记录不完整时，根据 `events.csv` 和 `session.yaml` 中的原因处理后重录；记录完整仍需满足求解器的有效帧数、时长和可辨识性检查。
4. **离线求解与复验。** 读取训练、验证会话，在终端检查状态、候选值、前后 RMS 和 `offline_validation_passed`。
5. **回填并在线验证。** 候选状态且独立复验通过后，手动回填运行配置的 `imu_delay_ms`。先运行 `handeye_test` 观察网格，再退出它并运行实际自瞄，检查典型负载下的等待时间、失败分布与动态效果。

低曝光、良好照明和有变化的运动有利于减少模糊与退化；Kalibr 官方也强调时间戳低抖动、运动激励和避免对称标定板位姿翻转。[采集原则](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)

工具已实现；可直接运行的 Linux 命令、三组测试结构和验收要求见 [v0.51 版本报告](RobotModel4_v0.51.md)。真实硬件 delay 尚需上车采集。实现以棋盘格内角点网格中心为坐标原点，减少整体角点顺序翻转带来的参考点变化；当前 delay 基线从采集配置读取。

### 6.4、Linux 仿真与上车数据

Linux 仿真用于验证采集接口、时间格式、正负号、插值、人工注入 delay 和丢包处理。给模拟图像与姿态注入已知的不同延迟，才能检验能否恢复其相对差；两路同时移动同样的时间不能验证相对标定。

仿真共享内存链路与真实海康、串口链路分别记录 timestamp_semantics 和配置。仿真最优 delay 不直接写入实车配置；实车结果也不强制用于仿真。

## 7、验收与反馈

### 7.1、软件验证

| 场景 | 预期 |
|---|---|
| 精确样本时刻、正常区间 | 返回有效单位四元数、正确时间与比例 |
| q 与 −q、近零旋转、跨 ±π | 姿态连续，最短旋转插值 |
| 非有限四元数、重复/乱序、时间回退 | 明确拒绝或按已定义规则处理，不产生 NaN 静默进入 EKF |
| 缺右样本、缓存前越界、超过最大间隔 | 正确失败状态；在线有等待上限；离线不外推 |
| 同一原始记录反复扫不同 delay | 结果可重复，不因队列被消费而变化 |
| 已知正、负、小数偏移 | 符号和单位正确，正负都可恢复 |
| 静止、理想固定轴匀速 | 识别不可辨识，不能宣称得到精确偏移 |
| 全部候选无效、最小值在边界 | 不写可部署参数，保留失败原因 |
| 录制拥塞与正常退出 | 计数可核对，已接受记录排空或明确标记丢失 |
| 时间语义、配置摘要不匹配 | 拒绝直接复用旧标定值 |

无噪声恢复测试应使用同一插值模型生成观测，预期误差不超过半个扫描步长加数值容差。若直接用连续运动真值采样，则需另外量化 100Hz 插值模型本身的误差，不能强行要求任意运动都达到相同步长精度。

### 7.2、实机验收

实机验收关注以下指标，其中扫描曲线与分段数据用于程序内部判定。

- 采集配置中的当前 delay、候选值、完整 $J(\Delta)$ 曲线和有效公共帧数。
- 标定板世界坐标 RMS、P95 距离残差，按完整动作片段分别统计。
- 各片段最佳值及最小区域是否重叠；若估计置信区间，说明时间块采样方法。
- IMU 接收间隔 P50/P95/P99、异常间隔、图像保存与丢弃数量。
- 在线插值间隔、实际等待时间、失败状态分布及有效对齐比例。
- 相机曝光/帧率、场景亮度、典型 CPU/GPU 负载和配置摘要。

相邻 IMU 接收间隔的分布只能反映到达规律，不能直接当作真实采样时刻误差的标准差。固定延迟与随机抖动应分别讨论。

验收判断为：独立数据上的改善超过片段间自然波动，曲线有可辨识最小区域，分段结果一致，并且在线查询的等待和覆盖率满足运行预算。仅训练集改善、最优值贴边、换速度明显漂移或等待超预算，均不能作为部署通过。

没有真实数据前不规定通用“亚毫秒精度”或提升百分比；绝对空间误差和延迟预算由第一批静止噪声、动作数据及运行需求共同确定。若独立数据不支持改善，保留原参数并记录原因。

### 7.3、对 EKF 的后续观察

时间对齐完成后，可比较动态场景的更新前创新、标准 NIS、关联失败和重建频率，确认改善是否传递到目标估计。目标运动、几何误差和检测噪声仍会影响这些量，因此不能用“延迟标定后 NIS 必然通过”作为前提。

不要以减少 EKF 拒绝次数作为延迟求解的直接目标。延迟标定首先满足固定场景的几何一致性，再独立观察其对 EKF 的作用。

## 8、横向对比与本项目取舍

| 方案 | 可借鉴点 | 不能直接继承的部分 |
|---|---|---|
| SP25 | 回退查询、四元数插值、世界网格叠加检查 | 固定 1ms 及仅靠肉眼确定最佳值 |
| 武科 | 棋盘格位置 RMS 自动搜索、连续记录与离线插值 | 既有补偿符号、硬编码板规格、主动运动指令、简化的有效性判定 |
| 南理本地版本 | 曝光事件与 IMU 共用板端时间、相机时钟关系检查 | 未核验的硬件条件及源码中的曝光中点/输出时间语义问题 |
| FYT / 中石大 | 配置化相对偏移、按图像时间查询姿态 | 各车型的毫秒值及“TF 等于硬件同步”的误解 |
| hhh v0.51 | 保留现有软件对齐，补齐完整采集、自动求解与复验 | 不能把相对 delay 当绝对曝光时刻或随机抖动的完整修复 |

本版形成“可解释、可采集、可重复求解、可拒绝无效结果”的时间对齐流程。后续源采样时间戳和硬件曝光同步是改进时间基准的方向，但不作为本次软件延迟标定的并列路线或已实现能力。

## 9、外部资料与使用边界

- [OpenCV PnP 文档](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html)：PnP 坐标变换定义、IPPE 共面要求及多解接口。
- [Kalibr 相机—IMU 标定说明](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)：空间/时间标定、采集激励、时间戳抖动与标定板对称性问题。本文借鉴采集原则，不声称仅有四元数的现有串口数据可以直接运行 Kalibr 的标准惯性标定。
- 各队行为以文中链接的本地源码为依据，不以源码中存在的算法推断实车实际精度，不引用 v0.50 的效果结论。
