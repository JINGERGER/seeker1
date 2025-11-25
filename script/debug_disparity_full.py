#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from stereo_msgs.msg import DisparityImage
import numpy as np

class DebugDisparityFull(Node):
    def __init__(self):
        super().__init__('debug_disparity_full')
        
        self.create_subscription(DisparityImage, '/front/disparity', self.debug_callback, 10)
        self.get_logger().info('Full disparity debug node started')
    
    def debug_callback(self, msg):
        # 转换完整图像
        data_uint8 = np.array(msg.image.data, dtype=np.uint8)
        disparity = data_uint8.view(dtype='<f4').reshape((msg.image.height, msg.image.width))
        
        # 统计分析
        unique_values, counts = np.unique(disparity, return_counts=True)
        
        self.get_logger().info(f'Image shape: {disparity.shape}')
        self.get_logger().info(f'Total unique values: {len(unique_values)}')
        
        # 显示最常见的值
        sorted_idx = np.argsort(counts)[::-1]  # 从多到少排序
        for i in range(min(10, len(unique_values))):
            idx = sorted_idx[i]
            val = unique_values[idx]
            count = counts[idx]
            percentage = count / disparity.size * 100
            self.get_logger().info(f'Value {val:.3f}: {count} pixels ({percentage:.1f}%)')
        
        # 参数信息
        self.get_logger().info(f'Parameters: f={msg.f}, T={msg.t}, min_disp={msg.min_disparity}, max_disp={msg.max_disparity}')
        
        # 寻找有效像素 (不是1023.984)
        invalid_marker = 1023.984375
        valid_mask = np.abs(disparity - invalid_marker) > 0.1
        valid_pixels = disparity[valid_mask]
        
        if len(valid_pixels) > 0:
            self.get_logger().info(f'Valid pixels: {len(valid_pixels)} ({len(valid_pixels)/disparity.size*100:.1f}%)')
            self.get_logger().info(f'Valid range: {valid_pixels.min():.3f} - {valid_pixels.max():.3f}')
            self.get_logger().info(f'Valid samples: {valid_pixels[:10]}')
        else:
            self.get_logger().warn('No valid pixels found!')
        
        # 只运行一次
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = DebugDisparityFull()
    rclpy.spin(node)

if __name__ == '__main__':
    main()