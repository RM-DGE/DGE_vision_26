#!/usr/bin/env bash
#
# 一键启动 bevy_robomaster_sim + sp25_learn(Foxglove) 联调。
# 用法:  ./run_foxglove.sh           (Ctrl-C 退出，会同时停止仿真器)
#        SIM_DIR=/path ALGO_DIR=/path ./run_foxglove.sh   (覆盖默认路径)
#
set -o pipefail

SIM_DIR="${SIM_DIR:-/home/ljy/桌面/bevy_robomaster_sim-master}"
ALGO_DIR="${ALGO_DIR:-/home/ljy/桌面/sp25_learn}"
BUILD_DIR="${BUILD_DIR:-$ALGO_DIR/build-talos-foxglove}"
CONFIG="${CONFIG:-$ALGO_DIR/configs/standard_sim.yaml}"
OPENVINO_SETUP="${OPENVINO_SETUP:-/home/ljy/opt/openvino_2024.6.0/setupvars.sh}"
DAEDALUS_BIN="${DAEDALUS_BIN:-$SIM_DIR/target/release/daedalus}"
STANDARD_BIN="${STANDARD_BIN:-$BUILD_DIR/standard}"
SHM_META="${SHM_META:-/tmp/talos_ipc_meta}"
STUDIO_PORT="${STUDIO_PORT:-8765}"
DAEDALUS_LOG="${DAEDALUS_LOG:-/tmp/daedalus_run.log}"

die() { echo "[run] 错误: $*" >&2; exit 1; }
info() { echo "[run] $*"; }

[ -x "$DAEDALUS_BIN" ] || die "找不到仿真器: $DAEDALUS_BIN（先 cargo build --release --bin daedalus）"
[ -x "$STANDARD_BIN" ] || die "找不到算法: $STANDARD_BIN（先构建 -DENABLE_FOXGLOVE=ON）"
[ -f "$CONFIG" ] || die "找不到配置: $CONFIG"

DAEDALUS_PID=""
STANDARD_PID=""
cleanup() {
  trap - INT TERM EXIT
  if [ -n "$STANDARD_PID" ] && kill -0 "$STANDARD_PID" 2>/dev/null; then
    info "停止算法 (pid $STANDARD_PID)..."
    kill "$STANDARD_PID" 2>/dev/null
  fi
  if [ -n "$DAEDALUS_PID" ] && kill -0 "$DAEDALUS_PID" 2>/dev/null; then
    info "停止仿真器 (pid $DAEDALUS_PID)..."
    kill "$DAEDALUS_PID" 2>/dev/null
  fi
  sleep 1
  [ -n "$STANDARD_PID" ] && kill -9 "$STANDARD_PID" 2>/dev/null
  [ -n "$DAEDALUS_PID" ] && kill -9 "$DAEDALUS_PID" 2>/dev/null
}
trap cleanup INT TERM EXIT

if pgrep -x daedalus >/dev/null 2>&1; then
  die "检测到正在运行的 daedalus，请先关闭它再执行本脚本"
fi
rm -f /tmp/talos_ipc_meta /tmp/talos_ipc_image_pool 2>/dev/null || true

info "启动仿真器: $DAEDALUS_BIN"
DISPLAY="${DISPLAY:-:0}" "$DAEDALUS_BIN" >"$DAEDALUS_LOG" 2>&1 &
DAEDALUS_PID=$!

info "等待共享内存 $SHM_META ..."
ok=""
for _ in $(seq 1 120); do
  [ -e "$SHM_META" ] && { ok=1; break; }
  kill -0 "$DAEDALUS_PID" 2>/dev/null || { echo "---- 仿真器日志 ----"; tail -20 "$DAEDALUS_LOG"; die "仿真器提前退出"; }
  sleep 0.5
done
[ -n "$ok" ] || { echo "---- 仿真器日志 ----"; tail -20 "$DAEDALUS_LOG"; die "等待共享内存超时"; }
sleep 1.5
info "共享内存就绪 ($SHM_META)"

if [ -f "$OPENVINO_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$OPENVINO_SETUP" >/dev/null 2>&1 || true
  info "已加载 OpenVINO: $OPENVINO_SETUP"
else
  info "警告: 未找到 $OPENVINO_SETUP，若报 OpenVINO 运行时错误请手动 source。"
fi

echo
info "Foxglove 连接: 打开 Foxglove Studio → Open connection → Foxglove WebSocket → ws://127.0.0.1:${STUDIO_PORT}"
info "  面板:  Image=/sp25/image/compressed   3D=/sp25/scene (frame: world)   Raw=/sp25/telemetry   TF=/tf"
info "  提示:  仿真窗口按 F5 开启外部自瞄跟随"
info "按 Ctrl-C 退出（会同时停止仿真器）"
echo

cd "$ALGO_DIR"
"$STANDARD_BIN" "$CONFIG" &
STANDARD_PID=$!
wait "$STANDARD_PID"
