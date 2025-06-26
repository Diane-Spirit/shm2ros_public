#!/usr/bin python3

import os
import mmap
import time
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

import shm2ros.shm_api as shm_api

class Shm2Ros(Node):

    def __init__(self):
        super().__init__("shm2ros")

        self.declare_parameter('shm_read_st', 0.01)
        self.declare_parameter('twist_pub_st', 0.01)
        self.declare_parameter('twist_topic', 'cmd_vel')
        self.declare_parameter('same_msg_limit', 3.)

        self.declare_parameter('shm_file', '/dev/shm/shared_control')
        self.declare_parameter('shm_file_size', 52)
        self.declare_parameter('shm_semaphore', '/dev/shm/semaphore_ros_read')
        self.declare_parameter('shm_semaphore_size', 5)
        
        self.shm_read_st = self.get_parameter('shm_read_st').get_parameter_value().double_value
        self.twist_pub_st = self.get_parameter('twist_pub_st').get_parameter_value().double_value
        self.twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        self.same_msg_limit = self.get_parameter('same_msg_limit').get_parameter_value().double_value

        self.shm_file = self.get_parameter('shm_file').get_parameter_value().string_value
        self.shm_file_size = self.get_parameter('shm_file_size').get_parameter_value().integer_value
        self.shm_semaphore = self.get_parameter('shm_semaphore').get_parameter_value().string_value
        self.shm_semaphore_size = self.get_parameter('shm_semaphore_size').get_parameter_value().integer_value
        
        self.read_shm_timer = self.create_timer(self.shm_read_st, self.read_shm)
        self.twist_pub = self.create_publisher(Twist, self.twist_topic, 10)
        self.get_logger().info(f" Publishing to {self.twist_topic} topic.")
        self.twist_pub_timer = self.create_timer(self.twist_pub_st, self.publish_twist)


        # Setting up shared file and semaphore
        self.shared_mem, self.flag_read_mem = shm_api.map_shared_memory(self.shm_file, self.shm_file_size, self.shm_semaphore, self.shm_semaphore_size)
        
        self.shm_data = [0.]*7
        self.twist_msg = Twist()

        self.last_consumed = 0
        self.same_msg_time = self.get_clock().now()

    def lock_acquire(self):
        t_ = self.get_clock().now()
        while self.flag_read_mem[0] == 1:
            # wait until semaphore is released
            if ((self.get_clock().now() - t_).nanoseconds / 1e9) > self.shm_read_st*(3/4):
                self.get_logger().warn(" Resource busy! Can't read the shm.")
                return 0
        self.flag_read_mem[0] = 1
        return 1
    
    def lock_release(self):
        self.flag_read_mem[0] = 0

    def read_shm(self):
        if self.lock_acquire():
            self.shared_mem.seek(0)
            self.shm_data = struct.unpack("<idddddd", self.shared_mem.read(shm_api.SHARED_CTRL_SIZE))
            self.lock_release()

    def update_last_consumed(self, latest_counter):
        if self.last_consumed != latest_counter:
            # Reset the time
            self.same_msg_time = self.get_clock().now()
        self.last_consumed = latest_counter

        if ((self.get_clock().now() - self.same_msg_time).nanoseconds / 1e9) > self.same_msg_limit:
            self.get_logger().warn(f" Got the same message in the last {self.same_msg_limit} seconds. Stopping robot.")
            return 0

        return 1

    def update_twist(self):
        # self.twist_msg.header.stamp = self.get_clock().now().to_msg()
        if self.update_last_consumed(self.shm_data[0]):
            self.twist_msg.linear.x = self.shm_data[1]
            self.twist_msg.angular.z = self.shm_data[6]
        else:
            self.twist_msg.linear.x = 0.
            self.twist_msg.angular.z = 0.

    def publish_twist(self):
        self.update_twist()
        self.twist_pub.publish(self.twist_msg)
    

def main():
    rclpy.init()
    shm2ros_node = Shm2Ros()

    rclpy.spin(shm2ros_node)

    shm2ros_node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()