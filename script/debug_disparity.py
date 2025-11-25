#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from stereo_msgs.msg import DisparityImage
import numpy as np
import struct

class DebugDisparity(Node):
    def __init__(self):
        super().__init__('debug_disparity')
        
        self.create_subscription(DisparityImage, '/front/disparity', self.debug_callback, 10)
        self.get_logger().info('Debug node started')
    
    def debug_callback(self, msg):
        # 检查原始数据
        raw_data = msg.image.data[:16]  # 前16个字节
        self.get_logger().info(f'Raw bytes: {raw_data}')
        
        # 尝试不同的解析方法
        byte_data = bytes(raw_data)
        
        # 小端序
        try:
            floats_le = struct.unpack('<4f', byte_data)
            self.get_logger().info(f'Little endian floats: {floats_le}')
        except:
            self.get_logger().info('Little endian failed')
        
        # 大端序
        try:
            floats_be = struct.unpack('>4f', byte_data)
            self.get_logger().info(f'Big endian floats: {floats_be}')
        except:
            self.get_logger().info('Big endian failed')
        
        # 使用 numpy 直接解析
        try:
            data_uint8 = np.array(raw_data, dtype=np.uint8)
            floats_np = data_uint8.view(dtype='<f4')  # 小端序 float32
            self.get_logger().info(f'NumPy little endian: {floats_np}')
        except Exception as e:
            self.get_logger().info(f'NumPy failed: {e}')
        
        # 参数信息
        self.get_logger().info(f'f={msg.f}, T={msg.t}, min_disp={msg.min_disparity}, max_disp={msg.max_disparity}')
        
        # 只运行一次
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = DebugDisparity()
    rclpy.spin(node)

if __name__ == '__main__':
    main()