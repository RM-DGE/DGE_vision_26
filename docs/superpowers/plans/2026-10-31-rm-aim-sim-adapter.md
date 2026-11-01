# rm_aim_sim Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the active Bevy/Daedalus simulator integration with a WSL-native `rm_aim_sim` protocol-v2 pixel closed loop while preserving production camera and CBoard behavior.

**Architecture:** A Linux-only `io::rm_aim_sim::LocalClient` owns the channel lock and all atomic JSON/raw-file IPC. `src/auto_aim_sim.cpp` consumes one owned BGR image plus same-frame pose per iteration, reuses the existing detector/solver/tracker/aimer/shooter pipeline, and sends absolute radian commands. Production `standard` and `sentry_DGE` return to direct hardware I/O; simulator truth remains a diagnostic-only type.

**Tech Stack:** C++17, CMake 3.16+, nlohmann-json, OpenCV, Eigen, POSIX `openat/flock/renameat`, yaml-cpp, WSL/Ubuntu 22.04, Godot `rm_aim_sim` protocol v2.

**Spec:** `docs/superpowers/specs/2026-10-31-rm-aim-sim-adapter-design.md`

## Global Constraints

- Work only on `codex/rm_aim_sim`, based on `test` commit `feaba15`; do not modify the user's current `test` worktree.
- Implement upstream `rm_aim_sim` commit `3895971a25e7712286395840b3c4cea9dffbc27c`, protocol string `rm_aim_sim/2`.
- Keep `rm_aim_sim` as an independent WSL repository; do not vendor its Godot source.
- Control must use rendered RGB/RGBA pixels and same-frame pose; `truth` is diagnostic-only.
- Build and final runtime validation in WSL-native paths, not `/mnt/c`.
- Every removal names one explicit file; never use recursive or batch deletion.
- Keep `CBoardBase` for ROS2 bridge/test injection and `CameraBase` for hardware camera implementations; remove only simulator-specific members.
- Use TDD for protocol, file IPC, clock mapping, and safety behavior.

---

### Task 1: Protocol-v2 typed parsing and command serialization

**Files:**
- Create: `io/rm_aim_sim/protocol.hpp`
- Create: `io/rm_aim_sim/protocol.cpp`
- Create: `tests/rm_aim_sim_protocol_test.cpp`
- Modify: `io/CMakeLists.txt`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `io::rm_aim_sim::SimContext`, `CameraModel`, `SimState`, `FrameMetadata`, `SimTruthTarget`, `parse_state(const nlohmann::json&)`, `parse_frame(const nlohmann::json&)`, and `make_command_json(const SimState&, int64_t, const io::Command&)`.
- Consumes: `io::Command` from `io/command.hpp` and protocol-v2 JSON.

- [ ] **Step 1: Add the failing protocol test target**

Add `enable_testing()` near the root `project(...)`, then add:

```cmake
add_executable(rm_aim_sim_protocol_test tests/rm_aim_sim_protocol_test.cpp)
target_link_libraries(rm_aim_sim_protocol_test PRIVATE io nlohmann_json::nlohmann_json)
add_test(NAME rm_aim_sim_protocol_test COMMAND rm_aim_sim_protocol_test)
```

The test must construct a complete minimal state with `protocol`, `type`, `publisher_id`, `session_id`, `publication`, `wall_unix_sec`, `sim_time`, `paused`, `appearance`, `gimbal`, `weapon`, `camera`, `truth`, and `ipc`, then assert exact typed values. It must also assert rejection of a wrong protocol, NaN/non-number angles, malformed K, dimensions outside 160..3840 by 120..2160, and an invalid frame filename.

- [ ] **Step 2: Run the test and confirm the missing-header failure**

Run in WSL:

```bash
cmake -S . -B build
cmake --build build --target rm_aim_sim_protocol_test -j2
```

Expected: compile failure because `io/rm_aim_sim/protocol.hpp` does not exist.

- [ ] **Step 3: Define focused protocol types**

Use these public shapes in `protocol.hpp`:

```cpp
namespace io::rm_aim_sim {
struct SimContext {
  std::string publisher_id;
  std::string session_id;
  bool operator==(const SimContext & other) const {
    return publisher_id == other.publisher_id && session_id == other.session_id;
  }
};
struct CameraModel {
  int width = 0;
  int height = 0;
  std::array<double, 9> K{};
  std::array<double, 5> distortion{};
  std::array<double, 9> R_camera2gimbal{};
  std::array<double, 3> t_camera2gimbal{};
  bool operator==(const CameraModel & other) const {
    return width == other.width && height == other.height && K == other.K &&
      distortion == other.distortion && R_camera2gimbal == other.R_camera2gimbal &&
      t_camera2gimbal == other.t_camera2gimbal;
  }
};
struct SimTruthTarget {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double yaw = 0;
  double vyaw = 0;
  bool is_outpost = false;
};
struct SimState {
  SimContext context;
  uint64_t publication = 0;
  double wall_unix_sec = 0;
  double sim_time = 0;
  bool paused = false;
  bool ipc_enabled = false;
  int64_t last_seq = -1;
  double bullet_speed = 0;
  Eigen::Quaterniond gimbal_to_world = Eigen::Quaterniond::Identity();
  CameraModel camera;
  std::vector<SimTruthTarget> truth;
};
struct FrameMetadata {
  SimContext context;
  std::string request_id;
  std::string frame_file;
  std::string encoding;
  size_t image_bytes = 0;
  double sim_time = 0;
  Eigen::Quaterniond gimbal_to_world = Eigen::Quaterniond::Identity();
  CameraModel camera;
};
SimState parse_state(const nlohmann::json & value);
FrameMetadata parse_frame(const nlohmann::json & value);
nlohmann::json make_command_json(
  const SimState & state, int64_t sequence, const io::Command & command);
}
```

- [ ] **Step 4: Implement strict parsing and serialization**

Use small helpers for finite numbers and fixed arrays. Enforce protocol/type, finite values, `[w,x,y,z]` quaternion normalization, dimensions, `rgb8/rgba8`, `image_bytes`, and exact `frame-<request_id>.raw`. `make_command_json` must emit booleans, copy current publisher/session, pass yaw/pitch unchanged (both DGE and protocol are down-positive pitch), set feed-forward fields to zero, and reject sequences outside `0..9007199254740991`.

- [ ] **Step 5: Run the protocol test**

Run:

```bash
cmake --build build --target rm_aim_sim_protocol_test -j2
ctest --test-dir build -R '^rm_aim_sim_protocol_test$' --output-on-failure
```

Expected: one passing test and zero failures.

- [ ] **Step 6: Commit Task 1**

```bash
git add CMakeLists.txt io/CMakeLists.txt io/rm_aim_sim/protocol.hpp io/rm_aim_sim/protocol.cpp tests/rm_aim_sim_protocol_test.cpp
git commit -m "feat: add rm_aim_sim protocol types"
```

---

### Task 2: Secure local IPC client and image handshake

**Files:**
- Create: `io/rm_aim_sim/local_client.hpp`
- Create: `io/rm_aim_sim/local_client.cpp`
- Create: `tests/rm_aim_sim_client_test.cpp`
- Modify: `io/CMakeLists.txt`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 protocol types/parsers.
- Produces: `ClientConfig`, `ReceivedFrame`, and `LocalClient::{poll_state,fresh,request_frame,try_read_frame,send_stop,send}`.

- [ ] **Step 1: Write failing temp-directory IPC tests**

The test fixture creates one explicit temporary directory with mode 0700 and fixture JSON/raw files. Cover:

```cpp
ClientConfig config;
config.channel = "test";
config.ipc_root = fixture.root();
config.state_timeout = 200ms;
config.frame_timeout = 5s;
LocalClient client(config);
assert(client.poll_state() == PollStateResult::updated);
client.request_frame();
assert(read_json(fixture.path("request.json"))["publisher_id"] == "publisher-a");
fixture.publish_matching_rgb_frame({255, 0, 0});
auto frame = client.try_read_frame();
assert(frame.has_value());
assert(frame->bgr.at<cv::Vec3b>(0, 0) == cv::Vec3b(0, 0, 255));
```

Also test: invalid channel, mode not 0700, second-client flock failure, repeated publication not refreshing freshness, mismatched request/publisher rejection, wrong raw length, RGBA conversion, stale state forcing stop, session change resetting sequence, and command JSON using `last_seq + 1`.

- [ ] **Step 2: Run and confirm the missing-client failure**

Run:

```bash
cmake --build build --target rm_aim_sim_client_test -j2
```

Expected: compile failure because `local_client.hpp` does not exist.

- [ ] **Step 3: Implement RAII directory and lock ownership**

`LocalClient` constructor must validate `[A-Za-z0-9_-]{1,80}`, open `<ipc_root>/rm_aim_sim_<channel>` using `O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW`, validate current UID and mode 0700, then open and non-blocking-flock `client.lock`. Store directory and lock fds; delete copy/move unless move is explicitly safe.

- [ ] **Step 4: Implement atomic JSON helpers**

Private helpers must:

```cpp
nlohmann::json read_json(const char * name) const;
void write_json(const char * name, const nlohmann::json & value) const;
static void read_all(int fd, void * output, size_t bytes);
```

Read only regular 1..65536-byte files via `openat(...O_NOFOLLOW)`. Write `<name>.<client-token>.tmp` completely, close it, then `renameat` to the mailbox. On failure, unlink only that exact temporary name.

- [ ] **Step 5: Implement state, request, frame, and command flow**

Use these public results:

```cpp
enum class PollStateResult { missing, unchanged, updated, context_changed };
struct ReceivedFrame {
  cv::Mat bgr;
  FrameMetadata metadata;
  std::chrono::steady_clock::time_point received_at;
};
```

`poll_state()` updates `last_state_received_` only for a new publication. `request_frame()` allows one outstanding request. `try_read_frame()` checks request/context, opens the exact raw file, validates type/size, copies pixels, converts to owned BGR, then clears the request. `send()` synchronizes sequence with `last_seq + 1`; if stale/paused/disabled, force both control and fire false. Destructor calls `send_stop()` in a no-throw block before closing fds.

- [ ] **Step 6: Run both IPC tests**

Run:

```bash
cmake --build build --target rm_aim_sim_protocol_test rm_aim_sim_client_test -j2
ctest --test-dir build -R '^rm_aim_sim_(protocol|client)_test$' --output-on-failure
```

Expected: both tests pass.

- [ ] **Step 7: Commit Task 2**

```bash
git add CMakeLists.txt io/CMakeLists.txt io/rm_aim_sim/local_client.hpp io/rm_aim_sim/local_client.cpp tests/rm_aim_sim_client_test.cpp
git commit -m "feat: add rm_aim_sim local IPC client"
```

---

### Task 3: Epoch-aware simulation clock

**Files:**
- Create: `io/rm_aim_sim/clock_mapper.hpp`
- Create: `io/rm_aim_sim/clock_mapper.cpp`
- Create: `tests/rm_aim_sim_clock_test.cpp`
- Modify: `io/CMakeLists.txt`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `SimContext` and frame `sim_time`.
- Produces: `MappedTime ClockMapper::map(const SimContext&, double, Clock::time_point)` and `ClockMapper::reset()`.

- [ ] **Step 1: Write failing epoch tests**

Assert that 1.000 then 1.010 seconds in the same context map exactly 10 ms apart; a publisher change, session change, or decreasing sim_time returns `epoch_changed=true` and anchors to the supplied receive time; non-finite or negative sim time throws.

- [ ] **Step 2: Run and confirm missing implementation**

Run:

```bash
cmake --build build --target rm_aim_sim_clock_test -j2
```

Expected: missing-header compile failure.

- [ ] **Step 3: Implement mapping**

Public API:

```cpp
using Clock = std::chrono::steady_clock;
struct MappedTime { Clock::time_point timestamp; bool epoch_changed; };
class ClockMapper {
public:
  MappedTime map(const SimContext &, double sim_time, Clock::time_point received_at);
  void reset();
private:
  std::optional<SimContext> context_;
  double base_sim_time_ = 0;
  double last_sim_time_ = 0;
  Clock::time_point base_local_{};
};
```

Keep a fixed base inside an epoch; do not re-anchor every frame. Treat a backwards delta greater than floating tolerance as a new epoch.

- [ ] **Step 4: Run the clock and all simulator tests**

Run:

```bash
cmake --build build --target rm_aim_sim_clock_test -j2
ctest --test-dir build -R '^rm_aim_sim_' --output-on-failure
```

Expected: protocol, client, and clock tests pass.

- [ ] **Step 5: Commit Task 3**

```bash
git add CMakeLists.txt io/CMakeLists.txt io/rm_aim_sim/clock_mapper.hpp io/rm_aim_sim/clock_mapper.cpp tests/rm_aim_sim_clock_test.cpp
git commit -m "feat: map rm_aim_sim epochs to steady time"
```

---

### Task 4: Pixel closed-loop executable and configuration

**Files:**
- Create: `src/auto_aim_sim.cpp`
- Create: `configs/rm_aim_sim.yaml`
- Modify: `CMakeLists.txt`
- Modify: `tools/Foxglove/foxglove.hpp`
- Modify: `tools/Foxglove/foxglove.cpp`
- Test: `tests/rm_aim_sim_client_test.cpp`

**Interfaces:**
- Consumes: `LocalClient`, `ClockMapper`, YOLO, Solver, Tracker, Aimer, Shooter.
- Produces: `auto_aim_sim` and diagnostic-only `publish_ground_truth(const std::vector<io::rm_aim_sim::SimTruthTarget>&)`.

- [ ] **Step 1: Add a failing command safety test**

Extend the client test so `fire=true` is serialized only when state is fresh, unpaused, IPC-enabled, and command.control is true. In every other combination assert `control=false` and `fire=false` in `command.json`.

- [ ] **Step 2: Run the safety test and verify failure**

Run the client test. Expected: at least the fire-without-control case fails before the guard is added.

- [ ] **Step 3: Create `rm_aim_sim.yaml`**

Start from algorithm values in `configs/daedalus.yaml`, rename the top-level simulator section to:

```yaml
rm_aim_sim:
  channel: default
  ipc_root: /dev/shm
  state_timeout_ms: 200
  frame_timeout_ms: 5000
  publish_truth: true
```

Keep virtual camera values matching `configs/vision.cfg`: 1440x1080, zero distortion, `fx=fy=1303.6753236814714`, `cx=720`, `cy=540`, optical-to-gimbal rotation `[0,0,1,-1,0,0,0,-1,0]`, and offset `[0.08,0,0.04]`. Set real-camera offsets/trims to zero.

- [ ] **Step 4: Implement the executable loop**

Create the client, detector, solver, tracker, aimer, shooter, and Foxglove server. Loop in this exact order: poll state; handle context/camera changes; request frame; try frame; map time; rebuild Tracker/Aimer on `epoch_changed`; set solver quaternion from frame metadata; detect; track; aim; compute shoot; send; publish diagnostics. Sleep 2 ms only when no frame is ready. Catch protocol/I/O exceptions, send stop, log, and retry only after a new state publication; fatal configuration mismatch exits nonzero.

- [ ] **Step 5: Add the executable target**

```cmake
add_executable(auto_aim_sim src/auto_aim_sim.cpp)
target_link_libraries(auto_aim_sim ${OpenCV_LIBS} fmt::fmt yaml-cpp nlohmann_json::nlohmann_json auto_aim tools io)
```

- [ ] **Step 6: Build and run simulator tests**

Run:

```bash
cmake --build build --target auto_aim_sim rm_aim_sim_client_test -j2
ctest --test-dir build -R '^rm_aim_sim_' --output-on-failure
```

Expected: executable builds and all simulator tests pass.

- [ ] **Step 7: Commit Task 4**

```bash
git add CMakeLists.txt src/auto_aim_sim.cpp configs/rm_aim_sim.yaml tools/Foxglove/foxglove.hpp tools/Foxglove/foxglove.cpp tests/rm_aim_sim_client_test.cpp
git commit -m "feat: add rm_aim_sim pixel closed loop"
```

---

### Task 5: Remove active Bevy/Daedalus integration one file at a time

**Files:**
- Modify: `io/camera.hpp`
- Modify: `io/camera.cpp`
- Modify: `io/cboard.hpp`
- Modify: `io/CMakeLists.txt`
- Modify: `src/standard.cpp`
- Modify: `src/sentry_DGE.cpp`
- Delete individually: `io/Daedalus/daedalus_camera.cpp`
- Delete individually: `io/Daedalus/daedalus_camera.hpp`
- Delete individually: `io/Daedalus/daedalus_cboard.hpp`
- Delete individually: `io/Daedalus/shared.hpp`
- Delete individually: `io/Daedalus/shm_client.hpp`
- Delete individually: `io/Daedalus/shm_layout.hpp`
- Delete individually: `io/Daedalus/shm_triple_buffer.hpp`
- Delete individually: `configs/daedalus.yaml`

**Interfaces:**
- Preserves: hardware `CameraBase::read`, `Camera`, `CBoardBase`, `CBoard`, ROS2 observers.
- Removes: `CameraBase::ground_truth`, `Camera::ground_truth`, Daedalus factory cases, and runtime CBoard selection.

- [ ] **Step 1: Add compile-regression expectations**

Ensure CMake still builds `standard`, `sentry_DGE`, `auto_aim_sim`, and `mcu_ros_delay_test` when ROS2 is available. Run `rg -n 'Daedalus|daedalus|bevy|talos' CMakeLists.txt io src configs` and save the pre-cleanup matches for comparison.

- [ ] **Step 2: Restore hardware-only entry points**

Remove Daedalus includes and YAML switching. `standard.cpp` and `sentry_DGE.cpp` construct `io::CBoard cboard(config_path);`. Remove truth publishing from hardware `standard`. Keep `CBoardBase` virtual methods and observer hooks because `McuRosBridge` and its fake board require them; update its comment to describe a hardware-board abstraction only.

- [ ] **Step 3: Narrow the camera abstraction**

Delete `SimTargetTruth`, `ground_truth()` virtual/member methods, and the `camera_name == "daedalus"` factory branch. Keep `CameraBase::read` so `HikRobot` remains an implementation and future hardware cameras can still plug in.

- [ ] **Step 4: Remove each legacy file with an explicit command**

Run one command per exact path, never a recursive directory deletion:

```powershell
Remove-Item 'io/Daedalus/daedalus_camera.cpp'
Remove-Item 'io/Daedalus/daedalus_camera.hpp'
Remove-Item 'io/Daedalus/daedalus_cboard.hpp'
Remove-Item 'io/Daedalus/shared.hpp'
Remove-Item 'io/Daedalus/shm_client.hpp'
Remove-Item 'io/Daedalus/shm_layout.hpp'
Remove-Item 'io/Daedalus/shm_triple_buffer.hpp'
Remove-Item 'configs/daedalus.yaml'
```

Do not remove the now-empty directory; Git does not track empty directories.

- [ ] **Step 5: Build all active targets and scan references**

Run:

```bash
cmake -S . -B build
cmake --build build --target standard sentry_DGE auto_aim_sim -j2
ctest --test-dir build --output-on-failure
rg -n 'Daedalus|daedalus|bevy|talos' CMakeLists.txt io src configs
```

Expected: build/tests pass; final `rg` has no active-code/config matches.

- [ ] **Step 6: Commit Task 5**

```bash
git add -A CMakeLists.txt io src configs tools/Foxglove
git commit -m "refactor: remove legacy Daedalus simulator path"
```

---

### Task 6: Current documentation and startup workflow

**Files:**
- Modify: `readme.md`
- Modify: `doc/README.md`
- Modify: `doc/EKF调试.md`
- Modify: `doc/融合说明.md`
- Modify: `foxglove_layouts/README.md`
- Create: `doc/rm_aim_sim调试.md`

**Interfaces:**
- Documents the two-repository WSL workflow, locked upstream commit, channel/config pairing, graphical camera requirement, and failure recovery.
- Leaves `doc/RobotModel4_v*.md` untouched as historical records.

- [ ] **Step 1: Write the runnable guide**

Include exact WSL-native clone/build/start commands, `./run.sh -- --config=res://configs/vision.cfg --remote`, `./build/auto_aim_sim configs/rm_aim_sim.yaml`, Foxglove URL, expected startup messages, and static/translation/spin scenarios. State explicitly that `--headless` cannot provide pixels.

- [ ] **Step 2: Replace active old-simulator instructions**

Update current docs and layout topic descriptions from Daedalus to `rm_aim_sim`. Preserve historical version documents. Explain that truth is scoring/visualization only.

- [ ] **Step 3: Validate commands and references**

Run:

```bash
rg -n 'Daedalus|daedalus|bevy_robomaster|talos' readme.md doc/README.md doc/EKF调试.md doc/融合说明.md foxglove_layouts/README.md
rg -n 'auto_aim_sim|rm_aim_sim.yaml|vision.cfg|--remote' readme.md doc/rm_aim_sim调试.md
```

Expected: no old active workflow references; all four new startup tokens are present.

- [ ] **Step 4: Commit Task 6**

```bash
git add readme.md doc/README.md doc/EKF调试.md doc/融合说明.md doc/rm_aim_sim调试.md foxglove_layouts/README.md
git commit -m "docs: document rm_aim_sim workflow"
```

---

### Task 7: WSL-native verification and end-to-end pixel loop

**Files:**
- Modify only if findings require it: `doc/rm_aim_sim调试.md`
- Verification artifacts: console output and simulator statistics; do not commit generated captures/build output.

**Interfaces:**
- Verifies every completion criterion from the design.

- [ ] **Step 1: Create WSL-native working copies**

Clone/check out the exact DGE branch under `~/Downloads/hhh-rm-aim-sim` and upstream commit `3895971a...` under `~/Downloads/sim/rm_aim_sim`. Do not delete existing directories; if either path exists, inspect it and use a new explicit sibling path.

- [ ] **Step 2: Run clean configure, build, and tests**

```bash
cd ~/Downloads/hhh-rm-aim-sim
source /opt/intel/openvino_2024/setupvars.sh
cmake -S . -B build
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

Expected: all active targets compile and all tests pass.

- [ ] **Step 3: Validate fixed-angle IPC before algorithms**

Start the graphical simulator with `vision.cfg --remote`, run its C++ demo for 30 frames, and confirm actual yaw/pitch approach the requested values with no rejected commands.

- [ ] **Step 4: Run static pixel closed loop**

Start `auto_aim_sim`, verify camera frames, pixel detections, same-frame quaternion, target tracking, command acceptance, firing, and nonzero resolved statistics. Confirm `truth` toggling changes only Foxglove diagnostic output, not command values.

- [ ] **Step 5: Run motion and safety scenarios**

Run translation and spin scenarios. Pause, disable remote control, stop the simulator, and restart it; each transition must produce stop/fire-false within 250 ms, a new epoch, and no reuse of the old target.

- [ ] **Step 6: Final repository verification**

```bash
git status --short
git diff test...HEAD --check
ctest --test-dir build --output-on-failure
rg -n 'Daedalus|daedalus|bevy|talos' CMakeLists.txt io src configs readme.md doc/README.md doc/EKF调试.md doc/融合说明.md foxglove_layouts/README.md
```

Expected: clean status after the final documentation commit, no whitespace errors, all tests pass, and no active legacy references.

- [ ] **Step 7: Commit any verification-only documentation correction**

If runtime verification changed commands or troubleshooting text, commit only those explicit docs:

```bash
git add doc/rm_aim_sim调试.md readme.md
git commit -m "docs: record rm_aim_sim verification"
```

If no correction is required, do not create an empty commit.
