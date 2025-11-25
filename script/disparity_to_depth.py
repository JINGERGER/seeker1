#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
import numpy as np
import struct
from rcl_interfaces.msg import SetParametersResult

class DisparityToDepth(Node):
    def __init__(self):
        super().__init__('disparity_to_depth')
        
        # 参数: enabled 控制是否发布深度图（默认关闭）
        self.declare_parameter('enabled', False)
        self.enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        # 支持运行时修改参数
        self.add_on_set_parameters_callback(self._on_parameter_change)
        # 订阅 4 个 disparity 话题 (使用正确的 DisparityImage 话题)
        self.create_subscription(DisparityImage, '/front/disparity', 
                                lambda msg: self.disparity_callback(msg, 'front'), 10)
        self.create_subscription(DisparityImage, '/right/disparity',
                                lambda msg: self.disparity_callback(msg, 'right'), 10)
        self.create_subscription(DisparityImage, '/back/disparity',
                                lambda msg: self.disparity_callback(msg, 'back'), 10)
        self.create_subscription(DisparityImage, '/left/disparity',
                                lambda msg: self.disparity_callback(msg, 'left'), 10)
        
        # 发布深度图话题
        self.depth_pubs = {
            'front': self.create_publisher(Image, '/front/depth/image_raw', 10),
            'right': self.create_publisher(Image, '/right/depth/image_raw', 10),
            'back': self.create_publisher(Image, '/back/depth/image_raw', 10),
            'left': self.create_publisher(Image, '/left/depth/image_raw', 10)
        }
        
        self.get_logger().info('Disparity to Depth node started')
    
    def ros_image_to_numpy(self, ros_image):
        """
        将 ROS Image (32FC1) 转换为 numpy 数组，避免使用 cv_bridge
        """
        if ros_image.encoding != '32FC1':
            raise ValueError(f"只支持 32FC1 编码，收到: {ros_image.encoding}")
        
        # 32FC1 = 32位浮点数，单通道
        height = ros_image.height
        width = ros_image.width
        
        # ROS Image 的 data 是字节数组，需要转换为 bytes 再解析
        if isinstance(ros_image.data, list):
            # 如果是整数列表，转换为字节
            byte_data = bytes(ros_image.data)
        else:
            # 如果已经是字节数据
            byte_data = ros_image.data
        
        # 从字节数据解析浮点数 (小端序)
        float_data = struct.unpack('<' + 'f' * (width * height), byte_data)
        
        # 重塑为图像形状
        image_array = np.array(float_data, dtype=np.float32).reshape((height, width))
        
        return image_array
    
    def numpy_to_ros_image(self, numpy_array, header):
        """
        将 numpy 数组转换为 ROS Image 消息，避免使用 cv_bridge
        """
        ros_image = Image()
        ros_image.header = header
        ros_image.height = numpy_array.shape[0]
        ros_image.width = numpy_array.shape[1]
        ros_image.encoding = '32FC1'
        ros_image.is_bigendian = False
        ros_image.step = ros_image.width * 4  # 4 bytes per float32
        
        # 将 numpy 数组转换为字节数据
        ros_image.data = numpy_array.astype(np.float32).tobytes()
        
        return ros_image
    
    def disparity_callback(self, msg, direction):
        """
        将 disparity 转换为 depth
        depth = (f * T) / disparity
        其中:
        - f: 焦距 (像素)
        - T: 基线距离 (米)
        - disparity: 视差值 (像素)
        """
        # 从 DisparityImage 获取参数
        f = msg.f  # 焦距
        T = msg.t  # 基线
        min_disp = msg.min_disparity
        max_disp = msg.max_disparity
        
        # 直接从 ROS Image 数据转换为 numpy 数组 (避免 cv_bridge)
        disparity = self.ros_image_to_numpy(msg.image)
        
        # 创建深度图
        depth = np.zeros_like(disparity, dtype=np.float32)
        
        # 检测无效标记值 (1023.984375 是 seeker 的无效标记)
        invalid_marker = 1023.984375
        
        # 有效的 disparity 值: 在合理范围内且不是无效标记
        valid_mask = (disparity > min_disp) & (disparity <= max_disp) & \
                     np.isfinite(disparity) & (np.abs(disparity - invalid_marker) > 0.1)
        
        # depth = (f * T) / disparity
        depth[valid_mask] = (f * T) / disparity[valid_mask]
        
        # 限制深度范围 (0.1m - 10m) 用于 RViz2 显示
        depth = np.clip(depth, 0.0, 10.0)
        
        # 无效区域设为 0 (RViz2 可以正常显示)
        depth[~valid_mask] = 0.0
        
        # 如果未启用深度发布，则跳过真正的发布（保留节点运行以便可动态使能）
        if not self.enabled:
            # 仅在首次记录时提示
            if not hasattr(self, f'_disabled_warn_{direction}'):
                setattr(self, f'_disabled_warn_{direction}', True)
                self.get_logger().info(f'depth publishing disabled (direction={direction}). Use ros2 param set /disparity_to_depth enabled true to enable.')
        else:
            # 转换为 ROS Image 消息 (避免 cv_bridge)
            depth_msg = self.numpy_to_ros_image(depth, msg.image.header)
            depth_msg.header.frame_id = msg.header.frame_id.replace('disparity', 'depth')
            
            # 发布深度图
            self.depth_pubs[direction].publish(depth_msg)
        
        # 打印统计信息（第一次）
        if not hasattr(self, f'_logged_{direction}'):
            setattr(self, f'_logged_{direction}', True)
            
            # 调试信息
            disp_min, disp_max = disparity.min(), disparity.max()
            disp_mean = disparity.mean()
            valid_count = np.sum(valid_mask)
            
            self.get_logger().info(
                f'{direction}: disparity range: {disp_min:.3f} - {disp_max:.3f}, mean: {disp_mean:.3f}, '
                f'valid pixels: {valid_count}/{disparity.size}'
            )
            
            valid_depths = depth[valid_mask]
            
            # 详细调试
            self.get_logger().info(f'{direction}: min_disp={min_disp}, max_disp={max_disp}')
            valid_disp = disparity[valid_mask]
            if len(valid_disp) > 0:
                self.get_logger().info(f'{direction}: sample valid disparity: {valid_disp[:5]}')
                sample_depth = (f * T) / valid_disp[:5]
                self.get_logger().info(f'{direction}: corresponding depth: {sample_depth}')
            
            if len(valid_depths) > 0:
                self.get_logger().info(
                    f'{direction}: f={f:.1f}px, T={T*1000:.1f}mm, '
                    f'depth range: {valid_depths.min():.3f}m - {valid_depths.max():.3f}m'
                )
            else:
                self.get_logger().warn(f'{direction}: No valid depth pixels found!')

    def _on_parameter_change(self, params):
        """Handle runtime parameter updates."""
        for p in params:
            if p.name == 'enabled':
                # p is rclpy.parameter.Parameter - use its .value
                try:
                    self.enabled = bool(p.value)
                except Exception:
                    self.enabled = False
                self.get_logger().info(f'disparity_to_depth: enabled -> {self.enabled}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = DisparityToDepth()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
