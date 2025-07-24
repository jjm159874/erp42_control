#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from struct import pack
import serial
import numpy as np

from interfaces_pkg.msg import ErpCmdMsg, ErpStatusMsg  # 수정: 실제 패키지 이름 확인

# from ByteHandler import Packet2ErpMsg, ErpMsg2Packet
from erp42_ws.ByteHandler import ErpMsg2Packet, Packet2ErpMsg

START_BITS = "535458"


class ERPHandler(Node):
    def __init__(self):
        super().__init__("erp_base")

        # Declare and get parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        self.get_logger().info(f"Serial Port: {port}")
        self.get_logger().info(f"Baudrate: {baudrate}")

        self.serial = serial.Serial(port=port, baudrate=baudrate)
        self.get_logger().info(f"Serial {port} connected.")

        self.alive = 0
        self.packet = ErpCmdMsg()
        self.packet.gear = 0
        self.packet.e_stop = False
        self.packet.brake = 1

        # ROS2 Publisher & Subscriber
        self.erpMotionMsg_pub = self.create_publisher(ErpStatusMsg, "/erp42_status", 3)
        self.create_subscription(ErpCmdMsg, "/erp42_ctrl_cmd", self.sendPacket, 10)

        # Timer (40Hz)
        self.create_timer(1.0 / 40.0, self.timer_callback)

    def recvPacket(self):
        try:
            packet = self.serial.read(18)
            if not packet.hex().find(START_BITS) == 0:
                end, data = packet.hex().split(START_BITS)
                packet = bytes.fromhex(START_BITS + data + end)
            msg = Packet2ErpMsg(packet)
            self.erpMotionMsg_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"recvPacket Error: {e}")

    def sendPacket(self, _data: ErpCmdMsg):
        self.packet = _data

    def serialSend(self):
        try:
            packet = ErpMsg2Packet(self.packet, self.alive)
            self.serial.write(packet)
            self.alive = (self.alive + 1) % 256
        except Exception as e:
            self.get_logger().warn(f"serialSend Error: {e}")

    def timer_callback(self):
        self.recvPacket()
        self.serialSend()


def main(args=None):
    rclpy.init(args=args)
    node = ERPHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()



