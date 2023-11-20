#!/usr/bin/env python3

import time
import rclpy
rclpy.init(args=None)

node = rclpy.create_node('waiting_node')
delay = node.declare_parameter('delay', 0.).value
what = node.declare_parameter('what', '').value

if what:
    node.get_logger().info(f'I am starting {what}')
node.get_logger().info(f'waiting for {delay} s')
time.sleep(delay)
node.get_logger().info('exit')

node.destroy_node()

rclpy.shutdown()
