#!/usr/bin/env python3
"""导航侧参考桥接节点。

职责：
  1. 订阅自瞄发布的 /mcu/imu、/mcu/status（导航定位/状态用）；
  2. 按导航决策发布 /enemy_status、/autoaim_target 给自瞄。

真实的 Nav2/行为树通常不会直接跑这个节点，而是把这里的
``enemy_ids`` / ``target_ids`` 换成行为树输出。这个节点主要用于：

  - 在没有完整导航栈时联调 ROS 链路；
  - 作为导航组接入 DGE 自瞄的最小示例；
  - 用 ``ros2 topic hz`` / ``ros2 topic delay`` 快速检查话题。

运行：
  source ~/dge_vision_ros2_ws/install/setup.bash
  ros2 run dge_nav_bridge nav_bridge --ros-args \
      -p enemy_ids:="2,5" -p target_ids:="1,3"
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from sp_msgs.msg import AutoaimTargetMsg, EnemyStatusMsg, McuStatus


class NavBridge(Node):
    def __init__(self):
        super().__init__("dge_nav_bridge")

        # 用逗号分隔的字符串而不是 int8[]，便于 ros2 run -p 直接传参。
        self.declare_parameter("enemy_ids", "")
        self.declare_parameter("target_ids", "")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("mcu_timeout_ms", 500.0)

        self.enemy_ids = self._parse_ids(self.get_parameter("enemy_ids").value)
        self.target_ids = self._parse_ids(self.get_parameter("target_ids").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.timeout_ms = float(self.get_parameter("mcu_timeout_ms").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.imu_sub = self.create_subscription(Imu, "/mcu/imu", self.on_imu, sensor_qos)
        self.status_sub = self.create_subscription(
            McuStatus, "/mcu/status", self.on_status, state_qos
        )

        self.enemy_pub = self.create_publisher(EnemyStatusMsg, "/enemy_status", state_qos)
        self.target_pub = self.create_publisher(AutoaimTargetMsg, "/autoaim_target", state_qos)

        self.last_imu_time = None
        self.last_status_time = None
        self.imu_count = 0

        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.publish_nav_info)
        self.get_logger().info(
            f"nav_bridge ready: enemy_ids={self.enemy_ids}, target_ids={self.target_ids}"
        )

    @staticmethod
    def _parse_ids(text: str):
        if not text:
            return []
        return [int(v) for v in text.replace(" ", "").split(",") if v]

    def on_imu(self, msg: Imu):
        self.last_imu_time = self.get_clock().now()
        self.imu_count += 1

    def on_status(self, msg: McuStatus):
        self.last_status_time = self.get_clock().now()

    def _mcu_ok(self) -> bool:
        if self.last_imu_time is None:
            return False
        age_ms = (self.get_clock().now() - self.last_imu_time).nanoseconds / 1e6
        return age_ms <= self.timeout_ms

    def publish_nav_info(self):
        # 导航数据过期时不要继续发旧目标，自瞄端也会做 1.5 s 超时。
        if not self._mcu_ok():
            return

        stamp = self.get_clock().now().to_msg()

        enemy = EnemyStatusMsg()
        enemy.timestamp = stamp
        enemy.invincible_enemy_ids = self.enemy_ids
        self.enemy_pub.publish(enemy)

        target = AutoaimTargetMsg()
        target.timestamp = stamp
        target.target_ids = self.target_ids
        self.target_pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = NavBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
