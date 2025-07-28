
# # erp42_cmd_sender.py
# import rclpy
# from rclpy.node import Node
# from interfaces_pkg.msg import ErpCmdMsg
# import serial
# import numpy as np
# from erp42_ws.ByteHandler import ErpMsg2Packet, Packet2ErpMsg 

# class ERP42CommandSender(Node):
#     def __init__(self):
#         super().__init__('erp42_cmd_sender')

#         self.ser = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.1)
#         self.alive_counter = 0

#         self.subscription = self.create_subscription(
#             ErpCmdMsg,
#             'erp42_cmd',
#             self.cmd_callback,
#             10
#         )

#         self.get_logger().info("ERP42 Command Sender Started")

#     def cmd_callback(self, msg: ErpCmdMsg):
#         self.alive_counter = (self.alive_counter + 1) % 256
#         packet = ErpMsg2Packet(msg, np.uint8(self.alive_counter))
#         self.ser.write(packet)
#         self.get_logger().info(
#             f"[CMD] Sent: speed={msg.speed}, steer={msg.steer}, brake={msg.brake}, gear={msg.gear}"
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     node = ERP42CommandSender()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import ErpCmdMsg, ErpStatusMsg

import threading


class ERP42Teleop(Node):
    def __init__(self):
        super().__init__('erp42_teleop')

        self.cmd_publisher = self.create_publisher(ErpCmdMsg, 'erp42_ctrl_cmd', 10)
        '''
        self.status_subscriber = self.create_subscription(
            ErpStatusMsg,
            'erp42_status',
            self.status_callback,
            10
        )
        '''
        self.get_logger().info("ERP42 Teleop Node Started (Spin Mode)")

        # 입력 쓰레드 시작
        self.input_thread = threading.Thread(target=self.command_loop, daemon=True)
        self.input_thread.start()

    def send_steer_cmd(self, steer_angle: int):
        msg = ErpCmdMsg()
        msg.e_stop = False
        msg.gear = 2
        msg.speed = 50     # 회전 가능하도록 속도 부여
        msg.steer = steer_angle
        msg.brake = 0
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"[COMMAND] Published steer: {steer_angle}")

    def status_callback(self, msg: ErpStatusMsg):
        self.get_logger().info(
            f"[STATUS] Speed: {msg.speed}, Steer: {msg.steer}, Brake: {msg.brake}, Gear: {msg.gear}"
        )

    def command_loop(self):
        while rclpy.ok():
            cmd = input("Enter command [left / right / straight / exit]: ").strip().lower()
            if cmd == 'left':
                self.send_steer_cmd(-2000)
            elif cmd == 'right':
                self.send_steer_cmd(2000)
            elif cmd == 'straight':
                self.send_steer_cmd(0)
            elif cmd == 'exit':
                rclpy.shutdown()
                break
            else:
                print("Invalid command.")


def main(args=None):
    rclpy.init(args=args)
    node = ERP42Teleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
