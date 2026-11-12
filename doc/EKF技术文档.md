# EKF技术文档：SP25、南理、武科、中南

本工程以同济 SP25 为基石，融合南理 `rmcs_auto_aim_v2` 的像素观测方案。本文沿用“状态与观测 → 过程模型 → P/Q/R → 关联 → NIS → 恢复”的结构，解释各实现的实际算法及本工程的优化空间。

核对日期：2026-11-03。

## 0、统一符号与基本公式

### 0.1、状态、观测函数与雅可比的区别

| 符号 | 含义 | 本文中的典型例子 |
|---|---|---|
| $\mathbf x$ | 待估计的状态向量 | 车心、速度、角度、角速度、半径、高差 |
| $\mathbf z$ | 本帧实测观测向量 | 装甲位姿、球坐标，或灯条像素 |
| $f(\mathbf x,dt)$ | 状态转移函数 | 三轴匀速、匀角速 |
| $F=\partial f/\partial\mathbf x$ | 状态转移雅可比 | 位置对速度的导数为 $dt$ |
| $h(\mathbf x)$ | 将状态映射到观测空间的函数 | 三维灯条端点投影到像素 |
| $H=\partial h/\partial\mathbf x$ | 观测雅可比 | 像素对车心、角度、半径的导数 |
| $P$ | 状态估计误差协方差 | 车心、速度等状态的不确定度 |
| $Q$ | 一次预测引入的过程噪声协方差 | 未建模加速度导致的不确定度 |
| $R$ | 观测噪声协方差 | 像素误差或PnP位姿误差 |

11维或13维描述的是状态维数，不能称为“11维观测”。$H$ 是矩阵，$h(\mathbf x)$ 才是非线性观测函数。$P,Q,R$ 的对角元素是方差，不是标准差；不同观测空间中的数值不能直接比较大小。

### 0.2、EKF预测与更新

$$
\mathbf x^-_k=f(\mathbf x^+_{k-1},dt),\qquad
P^-_k=F_kP^+_{k-1}F_k^T+Q_k
$$

$$
\nu_k=\mathbf z_k-h(\mathbf x^-_k),\quad
S_k=H_kP^-_kH_k^T+R_k,\quad
K_k=P^-_kH_k^TS_k^{-1}
$$

$$
\mathbf x^+_k=\mathbf x^-_k\boxplus K_k\nu_k
$$

其中 $\boxplus$ 表示按状态定义修正：普通状态相加，角度需归一化，SO(3)姿态需通过旋转群运算注入误差。角度观测的残差也应取最短角差。

Joseph形式的协方差更新为：

$$
P^+=(I-KH)P^-(I-KH)^T+KRK^T
$$

标准NIS使用更新前创新与协方差：

$$
\operatorname{NIS}=\nu^TS^{-1}\nu
$$

四维观测对应的卡方95%、99%分位数约为9.488、13.277。只有噪声模型、关联与线性化等假设基本成立时，才能按该分布解释统计量；同帧多条NIS的最大值也不等于一次四维检验。

### 0.3、两种加速度噪声离散化

后文对一个位置—速度对使用以下简写：

$$
B_D(q,dt)=q\begin{bmatrix}dt^4/4&dt^3/2\\dt^3/2&dt^2\end{bmatrix}
$$

$$
B_C(q,dt)=q\begin{bmatrix}dt^3/3&dt^2/2\\dt^2/2&dt\end{bmatrix}
$$

$B_D$ 表示每个时间段内随机恒定加速度模型（DWNA）；$B_C$ 表示连续白噪声加速度模型（CWNA）的离散协方差。两者的 $q$ 单位与含义不同，不能直接复制参数做公平比较。

## 1、SP25

### 1.1、设计状态变量、观测函数h、雅可比H与观测Z

观测流程：

```text
检测四角点
→ 每块装甲板执行IPPE PnP
→ camera → gimbal → world坐标变换
→ 固定位置、按板pitch先验搜索最优装甲yaw
→ 得到世界系视线yaw / pitch / distance / 装甲yaw
→ 预测整车状态并关联预测装甲板ID
→ 每块观测板执行一次4维EKF更新
```

PnP利用已知物点及其图像对应点估计位姿。普通装甲默认pitch先验为+15°，前哨为−15°；平衡目标不执行这项yaw优化。搜索范围为云台yaw附近±70°，步长1°，实际成本为四角重投影距离之和。

状态为：

$$
\mathbf x=[c_x,v_x,c_y,v_y,c_z,v_z,a,\omega,r,l,h]^T\in\mathbb R^{11}
$$

$r$ 是参考板组的半径，不保证一定为短半径；$l=r_2-r_1$ 可以有符号，$h=z_2-z_1$ 是另一板组的高度差。对 $N=4$ 的奇数板使用 $r+l,c_z+h$；其他板使用 $r,c_z$。

$$
a_i=\operatorname{wrap}(a+i2\pi/N),\qquad
\mathbf p_i=(c_x-r_i\cos a_i,\ c_y-r_i\sin a_i,\ z_i)^T
$$

定义 $g(\mathbf p)=[\operatorname{atan2}(p_y,p_x),\operatorname{atan2}(p_z,\sqrt{p_x^2+p_y^2}),\|\mathbf p\|]^T$，则：

$$
h_i(\mathbf x)=[g(\mathbf p_i)^T,a_i]^T,\qquad
\mathbf z=[yaw_{LOS},pitch_{LOS},d,yaw_{armor}]^T
$$

这里的视线yaw与装甲朝向yaw是不同物理量。H为4×11解析链式雅可比：

$$
H_i=\begin{bmatrix}J_g&0\\0&1\end{bmatrix}
\frac{\partial[\mathbf p_i^T,a_i]^T}{\partial\mathbf x}
$$

例如 $\partial p_x/\partial a=r_i\sin a_i$，$\partial p_y/\partial a=-r_i\cos a_i$；对r的导数为 $(-\cos a_i,-\sin a_i,0)^T$，奇数板对l有同样导数、对h的z导数为1。

依据：[Tracker观测入口](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/tracker.cpp:266>)、[PnP与坐标变换](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/solver.cpp:55>)、[h与H](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/target.cpp:187>)。

### 1.2、设计过程模型F

三轴匀速、车体匀角速，半径和高差保持不变：

$$
\mathbf c'=\mathbf c+\mathbf vdt,\qquad a'=\operatorname{wrap}(a+\omega dt)
$$

F为11×11单位阵加 $F_{0,1}=F_{2,3}=F_{4,5}=F_{6,7}=dt$（索引从0开始）。预测和状态修正后都归一化a；观测残差的第0、1、3维取最短角差。

前哨特殊处理：更新数大于10、半径合法且 $|\omega|>2$ 时，在预测前将角速度设为同符号的2.51 rad/s。

### 1.3、设计P、R、Q

普通四板车初始化：

$$
P_0=\operatorname{diag}(1,64,1,64,1,64,0.4,100,1,1,1)
$$

初始半径0.2m，中心由板位置加半径方向反推；速度、角速度、半径差、高差为0。平衡为2板；前哨为3板、半径0.2765m，半径P为 $10^{-4}$、l/h的P为0，z速度P为81。

$$
R=\operatorname{diag}\left(0.004,0.004,
\log(|\delta|+1)+1,\frac{\log(|d|+1)}{200}+0.09\right)
$$

$\delta=\operatorname{wrap}(yaw_{armor}-yaw_{LOS})$。距离通道方差随斜视角变化；装甲yaw方差随距离变化。该R是经验几何调度，没有将检测像素协方差通过PnP传播，也没有使用检测置信度。

Q中三组位置—速度块用 $B_D(100,dt)$，角度—角速块用 $B_D(400,dt)$，r/l/h噪声为0。前哨对应参数为10和0.1。P更新使用Joseph形式。

依据：[F与Q](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/target.cpp:75>)、[初始化分型](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/tracker.cpp:231>)、[R构造](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/target.cpp:192>)。

### 1.4、门控与数据关联

将预测板按距离排序，从最近三板中最小化：

$$
|\operatorname{wrap}(yaw_{armor}^{obs}-yaw_{armor}^{pred})|
+|\operatorname{wrap}(yaw_{LOS}^{obs}-yaw_{LOS}^{pred})|
$$

没有关联误差上限，也没有更新前马氏距离门。所有同name/type观测板依次更新一个EKF，不强制不同观测占用不同板ID。后面的观测使用前一次更新后的状态重新关联。

代码固定遍历前三板，但平衡模型只有两板，存在访问第三元素的静态越界路径；这是源码风险，本文未运行触发。

### 1.5、NIS与拒绝观测

代码在完成更新后才计算：

$$
r=z-h(x^+),\qquad S=HP^+H^T+R,\qquad nis=r^TS^{-1}r
$$

因此它是后验残差统计，不是标准更新前NIS，也不会拒绝当前观测。阈值为0.711，与注释中的“四维95%”不一致。普通 `track()` 在最近最多100次更新中累计失败达到40次时置lost，不要求先填满窗口；另一个全向感知重载没有同样检查。

代码中名为NEES的量使用本次状态修正差，不含真值，也不能当作标准状态估计NEES解释。依据：[滤波与统计实现](<D:/1RM_vision/sp25/同济-2025/tools/extended_kalman_filter.cpp:44>)。

### 1.6、时间、收敛与重建

dt来自图像时间戳差；Tracker在帧间隔大于0.1s时置lost，可在同帧重新建模。无同类板时先预测，再进入temp_lost；超过配置丢失帧数才lost。发散主要检查r及r+l是否都在0.05～0.5m。普通目标更新数>3且未发散后标收敛，前哨要求>10；不是协方差收敛判据。

## 2、南理rmcs_auto_aim_v2

### 2.1、设计状态变量、h/H/Z

普通机器人观测流程：

```text
检测完整装甲四角点
→ 首次用Robust PnP初始化中心和朝向
→ 预测整车状态
→ 生成4板、8灯条的3D端点
→ world/odom → camera → 像素投影
→ 完整板作锚、分配灯条ID
→ 每灯条一次4维像素EKF更新
```

$$
\mathbf x=[c_x,c_y,c_z,v_x,v_y,v_z,a,\omega,r_f,r_l,h_l]^T
$$

令 $u=(\cos a,\sin a,0)^T$，$v=(-\sin a,\cos a,0)^T$，板中心为：

$$
p_0=c+r_fu,\quad p_1=c+r_lv+h_le_z,\quad
p_2=c-r_fu,\quad p_3=c-r_lv+h_le_z
$$

每板按板姿态、宽度和0.056m灯条高度生成上下端点，普通板pitch固定+15°。状态a表示中心到参考板方向，与SP25同名a的几何方向约定不能直接混用。

$$
z_j=[u_{top},v_{top},u_{bottom},v_{bottom}]^T
$$

$h_j(x)$ 为端点经相机外参和带畸变投影得到的四维像素。H为4×11：中心、半径、高差导数解析；角度的三维端点导数采用 $10^{-4}$ 中心差分，再乘相机旋转和针孔投影导数。速度、角速度对应H列为0，通过P中的互协方差被间接修正。

投影h含畸变，H的投影部分没有畸变导数；非零畸变下属于近似线性化。依据：[状态与几何](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:47>)、[观测投影](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:123>)、[H实现](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:234>)。

### 2.2、设计过程模型F

中心三轴匀速，a匀角速；F为单位阵加四个位置—速度、角度—角速的dt项。Q在每次predict时直接相加，不随dt缩放。

### 2.3、设计P、R、Q

南理原始普通模型的初始P为：

$$
P_0=\operatorname{diag}(64,64,64,64,64,64,0.4,100,0.01,0.01,0.01)
$$

该P不同于hhh。初始化选择水平像素宽度最大的板PnP，参考半径均设0.2m，高差、速度、角速度为0；比较a与a+π的完整可见板数，选择更多的一支。

$$
Q=\operatorname{diag}(10^{-4},10^{-4},10^{-8},0.2,0.2,10^{-8},10^{-3},1,10^{-8},10^{-8},10^{-8})
$$

$$
R=40I_4\quad(\mathrm{pixel}^2)
$$

40是方差，对应单分量标准差约6.32px。固定Q按预测调用次数累加，因此不同帧率下每秒注入的噪声不同。P更新采用Joseph形式并强制对称化；每次更新后重新投影，供下一灯条使用。

依据：[P/Q/R](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:71>)、[预测](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:418>)、[逐灯条更新](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:444>)。

### 2.4、门控与数据关联

朝向相机夹角小于配置70°的板两灯可见；70°～90°只标一灯可见；≥90°不可见。遍历检测板与完整可见预测板，选择四角平方像素误差最小的一对作anchor，无最大误差门。

观测板拆成灯条并加入额外灯条，按上端点x排序，从anchor向两侧以±1模8传播ID，再按序更新。接口支持散灯；当前检测主链的普通散灯提取被注释，不能把接口能力视为已启用观测。

依据：[关联实现](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:525>)。

### 2.5、NIS与拒绝观测

普通RobotModel没有计算NIS或使用卡方门。H、S逆矩阵或后验状态数值非法时跳过该灯条；半径夹在[0.1,0.4]m、高差夹在[−0.2,0.2]m，vx/vy夹在[−10,10]m/s。错误关联但数值合法的观测仍可进入更新。

### 2.6、时间、收敛与重建

按设备类别保存模型；只有本帧看到该类目标时才predict/correct，dt覆盖距该类上次seen的间隔。超过1.5s未seen则销毁。收敛要求correct流程数≥10、Pxx/Pyy≤1、初始化墙钟时间≥0.1s。状态/速度/角速或位置协方差越界则销毁，后续重新初始化。

南理另有前哨专用5维模型 $[c_x,c_y,c_z,\omega,\theta]^T$：由PnP生成中心xyz和参考板方向观测，通过固定参考板与三板高差补偿处理换板。它不是这里的11维像素RobotModel。依据：[前哨模型](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/outpost.cpp:208>)。

## 3、武科

### 3.1、设计状态变量、h/H/Z

实际采用13维误差状态迭代滤波器 `ErrorStateEKF`，没有调用同目录中的AdaptiveEKF或UKF。

```text
检测灯条端点
→ 转成灯条方向、中点、长度
→ 预测并投影候选板灯条
→ 一对一关联检测板与预测板，补充散灯
→ 堆叠同帧全部观测
→ 迭代误差状态更新
```

$$
x=[c_x,v_x,c_y,v_y,c_z,v_z,\rho_z,\omega,\log r_1,\log r_2,h,\rho_y,\rho_x]^T
$$

$\rho$ 是SO(3)旋转向量，不是三个欧拉角。普通车支持整车倾斜；前哨和基地仅取绕z旋转。半径采用指数映射保持正值。

每灯条观测为：

$$
z=\left[\operatorname{atan2}(u_t-u_b,v_t-v_b),
\frac{u_t+u_b}{2},\frac{v_t+v_b}{2},
\sqrt{(u_t-u_b)^2+(v_t-v_b)^2}\right]^T
$$

状态→三维板姿态→灯条端点→带畸变像素投影→上述四维变换，构成h。当前 `MEASURE_NORMALIZED=false`，中点和长度单位为像素，方向单位为弧度；角残差归一化。

完整板贡献两根灯条8维；恰好只匹配一块板时，还通过IPPE构造左右灯中心的相机深度差1维辅助观测。该辅助量与端点来自同一组角点，存在相关性，不能视为独立新增传感器信息。

依据：[状态与模型](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/motion_model.hpp:20>)、[观测定义](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/motion_model.hpp:269>)、[实际update入口](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_target.cpp:345>)。

### 3.2、设计过程模型F

中心匀速，整车姿态右乘 $\operatorname{Exp}([0,0,\omega dt]^T)$。误差注入采用 $R^+=R\operatorname{Exp}(\delta\rho)$，状态差采用 $\operatorname{Log}(R_{nom}^TR_{value})$。

F对“误差注入→状态预测→状态差”使用Ceres Jet自动微分。实际 `update_multi` 的H采用误差空间中心差分，步长 $10^{-6}$，每个扰动重新注入姿态并投影；不能将它概括成所有雅可比均自动微分。

### 3.3、设计P、R、Q

P0按状态分组：位置=1、速度=10、旋转向量=1、log半径及高差=1、角速度=100，全对角。普通目标初始半径0.26m，PnP板姿态乘板—车体逆变换得到中心与整车姿态。

车体系加速度噪声先变换到odom：

$$
A=R_{car}\operatorname{diag}(q_x,q_y,q_z)R_{car}^T
$$

$$
Q_{pp}=dt^4A/4,\quad Q_{pv}=dt^3A/2,\quad Q_{vv}=dt^2A
$$

绕z角度—角速使用DWNA块；倾斜姿态另有随机游走。log半径噪声为 $q_r/r^2$、高度为 $q_h$，这些几何噪声代码中不乘dt。

设实测灯长为L：

$$
R=\tfrac12\operatorname{diag}(\sigma_\alpha^2,(k_cL)^2,(k_cL)^2,(k_lL)^2)
$$

例如 `wit.yaml`：$q_{xyz}=[30,30,1]$、$q_{yaw}=30$、$k_c=0.2$、$k_l=0.3$、$\sigma_\alpha=0.1$，迭代5轮；其他车辆配置不同。这是几何/尺度调度，不是按创新统计在线估计Q/R。

同帧观测堆叠、R取块对角；迭代重算h/H，通过LDLT求K，最终用Joseph更新并对称化P。实际迭代式及SO(3)注入后的协方差处理仍需独立审查，不能直接认定为标准IEKF实现。

依据：[Q](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_target.cpp:280>)、[R](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_target.cpp:371>)、[迭代核心](<D:/1RM_vision/武科/awakening-main/3rdparty/KalmanHyLib/error_state_extended_kalman_filter.hpp:333>)、[配置例](<D:/1RM_vision/武科/awakening-main/config/wit.yaml:91>)。

### 3.4、门控与数据关联

候选板ID按正对相机程度取前三个；代价为中心像素距离、四边方向差及周长相对差的加权和。一对一贪心匹配，未成熟和成熟目标用不同门限。散灯另检查灯长比例、方向和端点位置门；要求已经匹配到完整板，不能仅靠散灯独立重获。

这与hhh的枚举一一匹配不同，但它的几何cost同样没有使用P。依据：[整板关联](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_target.cpp:508>)、[散灯关联](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_target.cpp:642>)。

### 3.5、NIS与拒绝观测

实际更新链未见NIS卡方门；主要通过关联前的几何门拒绝。预测中限制半径范围，部分状态越界时直接设零。源码还存在静态风险：`DIFFZ_N=1`，深度差R回调却访问 `r(1,1)`；R回调引用捕获局部角点数据也有生命周期风险。这些尚未运行复现，不能当成可直接搬用的成熟实现。

### 3.6、时间、收敛与恢复

按实际秒数判定暂丢超时，TEMP_LOST期间继续预测；当前目标暂丢时维护备用target，备用进入TRACKING后替换当前；当前恢复则放弃备用。普通换板通过固定板ID表达，不手工交换半径或重建车心。

这一备用假设机制适合参考；更复杂的13维状态与迭代次数是否有益，取决于目标倾斜程度、观测质量和算力。依据：[双target恢复](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_tracker.cpp:22>)。

## 4、FYT2024

### 4.1、设计状态变量、h/H/Z

```text
检测/PnP并可选优化板yaw
→ 按图像时间TF至odom
→ 预测整车状态
→ 同ID位置最近关联及位置/yaw双门
→ 每帧最多一次装甲xyz+yaw更新
```

实际为10维，不能据构造函数中的临时9维向量判断模型维数：

$$
x=[c_x,v_x,c_y,v_y,c_z,v_z,\psi,\omega,r,dz_c]^T
$$

另一半径 `another_r`、板组高度差在滤波器外维护。

$$
h(x)=[c_x-r\cos\psi,c_y-r\sin\psi,c_z+dz_c,\psi]^T,\quad
z=[x_a,y_a,z_a,yaw_a]^T
$$

F和H由Ceres Jet自动微分生成。$c_z$ 与 $dz_c$ 在单板观测中只以和出现，不能仅凭一个瞬间观测将二者独立确定。

依据：[状态和h](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_auto_aim/armor_solver/include/armor_solver/motion_model.hpp:32>)、[Tracker](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_auto_aim/armor_solver/src/armor_tracker.cpp:78>)。

### 4.2、设计过程模型F

普通车匀速+匀角速；前哨切换为只旋转，平移速度设0、中心保持。图像时间戳差提供dt。普通模型F包含位置—速度及yaw—角速的dt项。

### 4.3、设计P、R、Q

构造EKF时 $P_0=I_{10}$；初始化半径0.26m，速度和高差设0。`setState`只写x，不重新初始化P。

Q按DWNA块构造，r和高差对角用 $qdt^4/4$。配置平移参数均0.05，yaw为1，r为80；高差参数未配置时用节点默认800。代码中z及yaw交叉项存在误用x噪声参数的问题。

$$
R=\operatorname{diag}(k_x|x_a|,k_y|y_a|,k_z|z_a|,r_\psi)
$$

配置系数为 $4\times10^{-4},4\times10^{-4},9\times10^{-4},5\times10^{-3}$。这只是世界坐标绝对值缩放；某坐标为0时该方向R趋零，不能视为可靠的距离/观测质量模型。

P采用简化 $(I-KH)P^-$ 更新。另有重要实现限制：predict更新 `P_pri` 后没有回写 `P_post`；连续漏测时状态逐帧前推，而P未同步逐帧累积传播。

依据：[Q/R构造](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_auto_aim/armor_solver/src/armor_solver_node.cpp:53>)、[EKF实现](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_utils/include/rm_utils/math/extended_kalman_filter.hpp:69>)、[参数](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_bringup/config/node_params/armor_solver_params.yaml:8>)。

### 4.4、门控与数据关联

仅同ID候选，取预测板三维位置最近者，再要求位置差<0.5m、yaw差<1rad（配置例）。只有一块同ID且yaw差大时进入换板：更新参考yaw、交换内外两半径、处理高度组；若位置仍严重不符则重设中心并清平移速度。

换板只修改x而不变换P；该分支也未将matched设true，所以仍可能进入TEMP_LOST或在DETECTING阶段LOST。依据：[换板实现](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_auto_aim/armor_solver/src/armor_tracker.cpp:206>)。

### 4.5、NIS与拒绝观测

没有NIS门，以位置/yaw硬门决定是否更新。yaw通过连续解包处理跨π，但解包函数修改历史yaw，候选遍历也调用它，存在候选顺序影响。

### 4.6、时间与恢复

暂丢继续预测，匹配恢复后返回TRACKING；超时转LOST。失联帧阈值由当前dt反推 `lost_time/dt`，不是累计真实失联时间。初始化、换板、半径限幅只修改x，旧P可能与新几何状态不一致。

可参考自动微分接口与显式换板事件，不应直接复用其P传播、R公式或重置逻辑。


## 5、本工程hhh

本节按 v0.61 代码描述。11 维状态、像素观测和 CWNA 沿用 v0.60；本次补齐初始几何协方差、分组学习确认与预测切板。版本改动见 [RobotModel4_v0.61](RobotModel4_v0.61.md)，同输入复测步骤见 [EKF 仿真测试指南](EKF仿真测试指南.md)。

### 5.1、设计状态变量、h/H/Z

本工程继承SP25工程框架和Solver，普通目标滤波核心改为南理式11维像素模型：

$$
x=[c_x,c_y,c_z,v_x,v_y,v_z,a,\omega,r_f,r_l,h_l]^T
$$

```text
检测四角点
→ 首次使用统一 IPPE 候选与约束像素优化解算板位姿，通过质量验收后初始化整车（详见 RobotModel4_v0.60.md 2.0）
→ 按图像时间预测整车状态与P
→ 四板/八灯条投影
→ 每条候选边的角点RMS门（学习中按P适度扩大）
→ 枚举四板×观测×交换的一一关联，歧义帧保留预测
→ 每灯条标准NIS门及EKF顺序更新
→ 按板组确认几何；预测命中时刻选择可用板
→ 连续拒绝时由恢复管理器窗口拟合候选接管，或判LOST
```

实际 $z_j=[u_t,v_t,u_b,v_b]^T$。虽然跟踪默认仍运行Solver PnP，但correct只消费像素角点，不消费板三维位姿；实际调用的独立散灯列表为空。

当前 h、H 共用 Brown-Conrady 畸变投影，H 使用解析几何导数和完整畸变链式导数；角度列由绕 z 轴的叉积计算。早期“h 含畸变、H 不含畸变导数”的描述已经不适用于当前实现。

依据：[初始化与实际输入](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/target.cpp:29)、[Tracker](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/tracker.cpp:259)、[H与投影](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/projection.cpp:19)。

### 5.2、设计过程模型F

中心匀速、a匀角速，F四个dt项。只接受有限正dt；状态未来查询通过副本外推，不修改原滤波器时间或P。普通滤波预测正确累计传播P。

### 5.3、设计P、R、Q

以 standard3/sentry 配置为例，配置给出的基础对角阵为：

$$
P_{\mathrm{config}}=\operatorname{diag}(0.04,0.04,0.01,4,4,0.01,0.1,4,0.001,0.001,0.001)
$$

v0.61 默认启用几何学习。对未知的 $r_f,r_l,h_l$，初始化时按允许区间 $[l,u]$ 相对初值 $g_0$ 的均方差设置下限：

$$
P_{0,gg}=\max\left(P_{\mathrm{config},gg},\frac{(u-l)^2}{12}+\left(\frac{l+u}{2}-g_0\right)^2\right)
$$

在默认半径范围 $[0.1,0.4]$、高差范围 $[-0.2,0.2]$，半径初值 0.2、高差初值 0 的情况下，最后三项实际为 $0.01,0.01,0.013333\ldots\ \mathrm{m}^2$。同一份 P 同时进入关联评分和 NIS，避免只放大候选门、却仍用过小 P 拒绝新板。只在初始化设置下限，不在拒绝后反复膨胀 P。

$$
R=40I_4
$$

Q只有CWNA：x/y块用 $B_C(1,dt)$，z块用 $B_C(10^{-4},dt)$，角度块用 $B_C(10,dt)$；半径、高差采用 $qdt$ 随机游走。v0.60起删除DWNA与DIAG_RATE对照分支（原用于与sp25/旧rmcs的Q消融），实测阶段只维护单一Q模型，避免不可达代码和配置歧义；历史模型推导见[RobotModel4_v0.33](<C:/Users/hhh/Desktop/hhh/doc/RobotModel4_v0.33.md>)。

同一轮清理删除了不再生效的配置键：`process_model`、`noise_x/y/z`、`noise_vx/vy/vz`、`noise_rotation_angle/speed`、`auto_reinit_after_no_update`、`max_no_update_frames`、`nis_max_consecutive_rejects`、`nis_reject_cov_inflation`。旧YAML里残留这些键会被忽略，不会报错。

初始中心来自水平板法线偏移0.2m，速度/角速度为0，两半径为0.2m，高差为0。本次不改变这些状态初值及 Q/R。更新使用Joseph形式加对称化，保持已有数值稳定性结构。

依据：[配置](C:/Users/hhh/Desktop/hhh/configs/standard3.yaml:39)、[P/Q构造](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp:407)、[Q传播](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp:607)。

### 5.4、门控与数据关联

相对南理增加左右灯条交换尝试，以适应图像左右与机器人灯条ID方向差异。v0.60起不再逐板选anchor再排序传播，而是一次性枚举"4块板 × 合法观测 × 是否交换"的一一匹配，取全局最优：

$$
e=\sqrt{\tfrac14\sum_{k=1}^4\|p_k^{obs}-p_k^{pred}\|^2},\qquad
T=\max(40\mathrm{px},0.5\,w_{pred})
$$

每条候选边先过 $e<T$ 的角点RMS门；$e$ 是四角欧氏误差的RMS，不是误差均值。组合代价（默认 miss/clutter 权重均为 30）为

$$
C=30\,n_{\text{miss}}+30\,n_{\text{clutter}}+\tfrac12\left(\mathrm{NIS}+\log\det S+m\log 2\pi\right)
$$

其中 $m=8n$ 是已选边的观测维数。若次优代价与最优之差小于 `association_ambiguity_gap`（2.0），标记为 ambiguous。v0.61 在这种情况下跳过本帧像素更新，保留预测；候选数量仍用于诊断，不能解释为实际接受更新。

学习中的板组使用有上限的候选门限，令 $H_8$ 为整板八维投影雅可比，$r$ 为单个像素分量的观测方差：

$$
T_{\mathrm{learn}}=\max\left(T,\min\left(120,3\sqrt{\operatorname{tr}(H_8P^-H_8^T)/4+2r}\right)\right)
$$

这里除以 4 与整板四个二维角点的 RMS 口径一致。它只限制候选枚举范围，不替代联合关联似然或后续四维 NIS。0/2、1/3 分组确认：至少 5 次完整合格更新、有效观测跨度至少 0.04 s、半径标准差不超过 0.02 m，侧板组还要求高差标准差不超过 0.01 m；两组均须已有观测。确认后恢复严格门限 T。计数只在无歧义、残差合格、合法观测解释充分时累积，不用空等时间完成确认。

合法观测要求 large 一致、四角有限且凸、上下端 y 序正确、与已有观测端点距离≥0.5px，且最多保留6条。该匹配只做关联，不依赖本帧EKF更新；NIS门在之后的逐灯条更新里。只要有同name/type检测，外层Tracker仍可视为found，即使EKF全拒绝；因此tracking状态不能证明EKF本帧有有效更新。

依据：[关联枚举](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/association.cpp:25)、[correct](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp:687)。

### 5.5、NIS与拒绝观测

每根灯条用更新前 $\nu,S$ 计算标准四维NIS。配置在至少10个成功融合帧且滤波时间≥0.1s后启用门控：监控阈值9.488，拒绝阈值13.277。NIS拒绝只拒观测、不改先验，也不再放大P；持续失配交给恢复管理器处理。v0.61 沿用这些门控规则，通过合理初始几何 P 支持较晚出现的第二组板，不为该组另开无门控通道。

三个语义细节：

1. 没有有效尝试（无匹配、关联为空）时，帧NIS发布 NaN 且 `nis_valid=false`，表示未执行检验，不能解释为残差良好。
2. `update_count_` 只在本帧至少接受一根灯条时增加（v0.60 起），等价于有效融合帧数；无匹配、边门拒绝或全部NIS拒绝只增加 `no_update_frames_`。
3. 帧NIS输出是本帧有效尝试的最大单灯条NIS；另有均值字段。最大值超过阈值的频率不能直接按单次四维卡方失败率解释。

依据：[NIS与更新](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp:743)。

### 5.6、时间、收敛与恢复

每次 correct 没有接受任何端点对时增加 `no_update_frames`，包括进入 correct 的空观测帧。RecoveryManager 区分没有检测和检测被拒绝；前者不累计其恢复拒绝帧数。v0.60 起不再按帧数自动重初始化，改由 RecoveryManager 按图像时间管理健康状态（INIT/HEALTHY/COASTING/DEGRADED/LOST）。

连续拒绝≥3帧且距上次接受≥0.04s时，用最近0.12s窗口（3~8帧先验）拟合候选。候选要连续2帧、跨度≥0.02s通过关联RMS、相对主模型改善比例、身份距离和未来位置方差检查才接管，接管时整体替换状态与P；0.15s内未接管则过期。超过0.5s无接受更新或状态越界判LOST。

收敛还检查最小correct数、连续无更新数、位置/速度/角速协方差和初始化时长。数值非法、状态越界或协方差超阈值触发发散；长期无接受更新由恢复管理器判LOST，不再走 `max_no_update_frames`。

v0.61 的恢复主模型评分、窗口关联和候选评分使用相同分组学习门限。新板组有完整更新且关联清晰时记为 `geometry_learning`，不连续累计为恢复失败；但不会借此刷新最近质量合格时间，原 LOST 时间限制保持有效。

旧版“单帧PnP重建清速度”的路径已随 `auto_reinit_after_no_update` 删除；接管后的速度、角速度来自窗口拟合，不是清零。依据：[RecoveryManager](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/recovery.cpp:212)、[Tracker的LOST判断](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/tracker.cpp:220)。

### 5.7、预测选板（v0.61）

Aimer 先外推处理/发弹延时，再迭代弹丸飞行时间，每次从当前后验重新预测到命中时刻。普通模式按 $|\omega|\le2\ \mathrm{rad/s}$ 判定，在可用板中维持锁定；旋转模式按转向和切入/切出角窗口选择板。高低速延时也按 $|\omega|$ 判断，正反转一致。

不再因 `jumped_` 为 false 就固定返回模型 0 号板。几何未确认时，仅允许最近成功更新的板参与；相应板组确认后才允许该组其他板参与预测切换。没有可用板、弹道无解或迭代未收敛时瞄准点无效，空目标会清除旧点。因而绿点可以在窗口空档消失，不能把“模型仍存在”直接解释为“当前一定有瞄准指令”。

## 6、横向对比与算法含义

| 维度 | SP25 | 南理普通车 | 武科 | FYT2024 | hhh |
|---|---|---|---|---|---|
| 状态维数 | 11 | 11 | 13 | 10 | 11 |
| 观测 | 球坐标+板yaw | 灯条端点像素 | 灯条角度/中点/长度，另有单板深度差 | 板xyz+yaw | 灯条端点像素 |
| PnP作用 | 每次观测 | 初始化 | 初始化及单板辅助 | 每次观测 | 初始化，跟踪结果不入EKF |
| F/H | 解析 | 解析+差分，H未含畸变导数 | F自动微分，实际H误差空间差分 | 自动微分 | 解析F/H，H含畸变导数 |
| Q主要形式 | DWNA | 固定每预测对角Q | 车体系各向异性DWNA | DWNA，有参数误用 | 默认CWNA |
| R调度 | 斜视角/距离 | 固定40I | 灯长及物理分量 | 世界坐标绝对值 | 固定40I配置例 |
| 标准更新前NIS | 否，后验统计 | 无 | 无 | 无 | 有 |
| 多观测更新 | 逐板 | 逐灯条 | 堆叠迭代 | 单板 | 逐灯条 |
| P数值更新 | Joseph | Joseph+对称 | Joseph+对称 | 简化形式 | Joseph+对称 |
| 关联 | 最近三板角差 | anchor+灯条排序 | 一对一几何代价 | 最近板+位置/yaw门 | 分组学习门限+枚举一一匹配+NIS门 |
| 恢复特点 | lost后重建 | 删除后重建 | 暂丢保留+备用假设 | 换板改状态 | 窗口候选接管，0.5s判LOST |

像素域方案避免将每帧PnP点估计直接当测量，但仍受标定、角点噪声、可观测性和关联影响。PnP域方案结构较直观，但PnP深度/姿态噪声及相关性必须体现在R中。两者不能只凭状态维数或滤波器名称判断精度高低。

