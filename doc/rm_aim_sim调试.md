# rm_aim_sim 像素闭环调试

分支 `codex/rm_aim_sim` 基于 `test`；调试器独立部署，协议锁定上游 `3895971a25e7712286395840b3c4cea9dffbc27c`。

终端 1（WSL 图形模式，`--headless` 无相机像素）：

```bash
cd ~/Downloads/sim/rm_aim_sim
./run.sh -- --config=res://configs/vision.cfg --remote
```

终端 2（WSL 原生文件系统）：

```bash
cd ~/Downloads/hhh-rm-aim-sim
source /opt/intel/openvino_2024/setupvars.sh
cmake -S . -B build
cmake --build build -j4
ctest --test-dir build --output-on-failure
./build/auto_aim_sim configs/rm_aim_sim.yaml
```

第二个可选参数为运行秒数，例如 `./build/auto_aim_sim configs/rm_aim_sim.yaml 30`。
默认通道 `default`；修改两边 channel 必须一致。相机默认 1440×1080，视场角 45°；内外参与 YAML 逐项校验，不一致时拒绝控制。

控制端申请 `request.json`，收到匹配的 `frame.json/raw` 后复制 RGB/RGBA 为 BGR，并把同帧姿态交给现有检测、PnP、跟踪、瞄准和射击模块。真值只送 Foxglove 对照，不参与控制。Foxglove 连接 `ws://localhost:8765`。

日志每秒输出 frames、armors、targets、control、fire 和 stats。确认 armors/targets 非零、控制指令被接收，再检查 fired/hit/resolved。传输成功不代表检测网络能够识别当前外观。

静止靶先验收，再通过界面启用平移和旋转（或使用上游 translate.cfg / spin.cfg）。暂停、切换外部控制、重启都会清空旧跟踪；状态 200ms 未更新或观察过旧时停控停火。缺图 5s 提示图形模式要求。Ctrl+C 退出会发送停止命令。

旧 Bevy/Daedalus 文件在此分支逐文件删除，可从 `test` 历史恢复。真机仍使用 `standard`、`sentry_DGE`。

2026-11-01 实测：完整 WSL 原生构建通过，协议/握手/watchdog 测试通过。静止靶像素闭环达到 19/20 结算命中、350 伤害、1 次击毁；平移场景 `translate.cfg`（1.5m/s）12s 样本为 4/32 结算命中、3 发仍在飞行，命中率 12.5%。平移中估计速度约 ±1.2–1.65m/s，观测年龄 21–38ms；换向附近发生估计超调和 EKF 重初始化，运动靶效果仍需单独优化现有算法。上述是本机样本，不是性能保证。

旋转场景 `spin.cfg` 的 12s 样本：156 帧，最后日志为 19/90 结算命中（21.1%）、2 发在飞行、350 伤害、1 次击毁；观测年龄 29–42ms。静止、平移、旋转均使用真实图像检测，不使用理想装甲点。日志统计采样时仍在飞行的弹丸不计入命中率分母。
