#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from interfaces_pkg.msg import ErpCmdMsg, ErpStatusMsg


class ParkingNode(Node):
    def __init__(self):
        super().__init__('parking_node')

        # 퍼블리셔: ERP42 제어 명령
        self.cmd_publisher = self.create_publisher(
            ErpCmdMsg,  
            '/erp42_ctrl_cmd',
            10
        )

        # 구독자: /parking_ok (bool 메시지)
        self.parking_ok_subscriber = self.create_subscription(
            Bool,
            '/parking_ok',
            self.parking_ok_callback,
            10
        )

        self.get_logger().info("🚗 Parking Node Started")

    def parking_ok_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("🟢 Parking signal received. Executing parking maneuver...")
            self.execute_parking()
        else:
            self.get_logger().info("🔴 Parking signal is False. Waiting...")

    def execute_parking(self):
        # 단순한 후진 주차 예시
        for i in range(20):  # 20번 반복 (주차 거리 제어)
            cmd = ErpCmdMsg()
            cmd.e_stop = False
            cmd.gear = 2           # 후진
            cmd.speed = 10         # 느린 속도
            cmd.steer = 0          # 직진
            cmd.brake = 0

            self.cmd_publisher.publish(cmd)
            self.get_logger().info(f"[PARKING] Sent command {i+1}/20")

            self.sleep_ms(100)

        # 마지막에 브레이크
        cmd = ErpCmdMsg()
        cmd.e_stop = False
        cmd.gear = 2
        cmd.speed = 0
        cmd.steer = 0
        cmd.brake = 1
        self.cmd_publisher.publish(cmd)
        self.get_logger().info("✅ Parking complete.")

    def sleep_ms(self, ms: int):
        """ROS-safe sleep in milliseconds."""
        self.get_clock().sleep_for(rclpy.duration.Duration(milliseconds=ms))


def main(args=None):
    rclpy.init(args=args)
    node = ParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
