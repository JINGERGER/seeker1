#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
import numpy as np

class DisparityVisualizer(Node):
    def __init__(self):
        super().__init__('disparity_visualizer')
        
        # 订阅 disparity 话题
        self.create_subscription(DisparityImage, '/front/disparity', 
                                lambda msg: self.visualize_callback(msg, 'front'), 10)
        self.create_subscription(DisparityImage, '/right/disparity',
                                lambda msg: self.visualize_callback(msg, 'right'), 10)
        self.create_subscription(DisparityImage, '/back/disparity',
                                lambda msg: self.visualize_callback(msg, 'back'), 10)
        self.create_subscription(DisparityImage, '/left/disparity',
                                lambda msg: self.visualize_callback(msg, 'left'), 10)
        
        # 发布可视化图像
        self.vis_pubs = {
            'front': self.create_publisher(Image, '/front/disparity_vis/image_raw', 10),
            'right': self.create_publisher(Image, '/right/disparity_vis/image_raw', 10),
            'back': self.create_publisher(Image, '/back/disparity_vis/image_raw', 10),
            'left': self.create_publisher(Image, '/left/disparity_vis/image_raw', 10)
        }
        
        self.get_logger().info('Disparity Visualizer started')
    
    def ros_image_to_numpy(self, ros_image):
        """转换 ROS Image 为 numpy 数组"""
        data_uint8 = np.array(ros_image.data, dtype=np.uint8)
        float_data = data_uint8.view(dtype='<f4')
        return float_data.reshape((ros_image.height, ros_image.width))
    
    def numpy_to_ros_image(self, numpy_array, header, encoding='mono8'):
        """转换 numpy 数组为 ROS Image"""
        ros_image = Image()
        ros_image.header = header
        ros_image.height = numpy_array.shape[0]
        ros_image.width = numpy_array.shape[1]
        ros_image.encoding = encoding
        ros_image.is_bigendian = False
        
        if encoding == 'mono8':
            ros_image.step = ros_image.width
            ros_image.data = numpy_array.astype(np.uint8).tobytes()
        
        return ros_image
    
    def visualize_callback(self, msg, direction):
        """将 disparity 转换为可视化图像"""
        # 获取参数
        min_disp = msg.min_disparity
        max_disp = msg.max_disparity
        
        # 转换 disparity 图像
        disparity = self.ros_image_to_numpy(msg.image)
        
        # 创建可视化图像 (8位灰度)
        vis_image = np.zeros_like(disparity, dtype=np.uint8)
        
        # 无效标记 (1023.984375)
        invalid_marker = 1023.984375
        
        # 有效像素: 不是无效标记且在合理范围内
        valid_mask = (disparity > min_disp) & (disparity <= max_disp) & \
                     (np.abs(disparity - invalid_marker) > 0.1)
        
        if np.sum(valid_mask) > 0:
            # 将有效 disparity 映射到 0-255
            valid_disp = disparity[valid_mask]
            disp_min, disp_max = valid_disp.min(), valid_disp.max()
            
            if disp_max > disp_min:
                # 归一化到 0-255
                normalized = (disparity - disp_min) / (disp_max - disp_min) * 255
                vis_image[valid_mask] = normalized[valid_mask].astype(np.uint8)
            else:
                # 所有有效值相同
                vis_image[valid_mask] = 128
        
        # 无效像素保持为 0 (黑色)
        
        # 转换为 ROS Image 消息
        vis_msg = self.numpy_to_ros_image(vis_image, msg.header)
        vis_msg.header.frame_id = msg.header.frame_id.replace('depth', 'disparity_vis')
        
        # 发布可视化图像
        self.vis_pubs[direction].publish(vis_msg)
        
        # 打印统计信息（第一次）
        if not hasattr(self, f'_vis_logged_{direction}'):
            setattr(self, f'_vis_logged_{direction}', True)
            valid_count = np.sum(valid_mask)
            invalid_count = np.sum(np.abs(disparity - invalid_marker) <= 0.1)
            
            self.get_logger().info(
                f'{direction}: total pixels: {disparity.size}, '
                f'valid: {valid_count}, invalid markers: {invalid_count}, '
                f'other: {disparity.size - valid_count - invalid_count}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = DisparityVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()