#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from studica_control.srv import SetData

class ServoRelayNode(Node):
    def __init__(self):
        super().__init__('servo_relay_node')
        # Parameter: name of the servo (must match config)
        self.declare_parameter('servo_name', 'servo1')
        self.servo_name = self.get_parameter('servo_name').get_parameter_value().string_value

        # Create client for the servo service
        service_name = f'/{self.servo_name}/set_servo_angle'
        self.cli = self.create_client(SetData, service_name)
        self.get_logger().info(f'Waiting for service {service_name}...')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {service_name} not available, exiting')
            rclpy.shutdown()
            return
        self.get_logger().info(f'Connected to service {service_name}')

        # Subscribe to the angle command topic
        topic_name = f'/{self.servo_name}/angle_cmd'
        self.sub = self.create_subscription(
            Int32,
            topic_name,
            self.cmd_callback,
            10)
        self.get_logger().info(f'Subscribed to topic {topic_name}')

    def cmd_callback(self, msg: Int32):
        angle = msg.data
        self.get_logger().info(f'Received angle command: {angle}')
        req = SetData.Request()
        req.params = str(angle)
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'Servo set: {resp.message}')
            else:
                self.get_logger().error(f'Service failed: {resp.message}')
        except Exception as e:
            self.get_logger().error(f'Service call exception: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ServoRelayNode()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
