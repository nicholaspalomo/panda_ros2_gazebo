#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity

# Borrowed from: https://bitbucket.org/theconstructcore/box_bot/src/foxy/box_bot_description/launch/spawn_box_bot.py
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_camera')
    spawn_entity_client = node.create_client(SpawnEntity, '/spawn_entity')

    content = ""
    if sys.argv[1] is not None:
        with open(sys.argv[1], 'r') as content_file:
            content = content_file.read()

    req = SpawnEntity.Request()
    req.name = "kinect"
    req.xml = content
    req.robot_namespace = "kinect"
    req.reference_frame = "world"

    while not spawn_entity_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = spawn_entity_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()