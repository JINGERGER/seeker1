#!/usr/bin/env python3
"""
Omnidirectional (MEI model) image undistortion node for ROS2
Properly handles the unified omnidirectional camera model with xi parameter
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class OmniUndistortNode(Node):
    def __init__(self):
        super().__init__('omni_undistort_node')
        
        # Declare parameters
        self.declare_parameter('config_file', 'seeker_omni_depth/kalibr_cam_chain.yaml')
        self.declare_parameter('scale', 0.5)
        self.declare_parameter('fov_scale', 1.5)  # >1.0: 视野变宽, <1.0: 视野变窄, 推荐: 1.2-2.0
        
        # Get parameters
        config_file = self.get_parameter('config_file').value
        self.scale = self.get_parameter('scale').value
        self.fov_scale = self.get_parameter('fov_scale').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load calibration data
        self.camera_params = {}
        self.load_calibration_file(config_file)
        
        # Define camera mappings
        self.camera_topics = {
            'cam0': '/fisheye/left/image_raw',
            'cam1': '/fisheye/right/image_raw',
            'cam2': '/fisheye/bright/image_raw',
            'cam3': '/fisheye/bleft/image_raw'
        }
        
        # Create subscribers and publishers
        self.image_subscribers = {}
        self.image_publishers = {}
        
        for cam_name, input_topic in self.camera_topics.items():
            output_topic = input_topic.replace('/fisheye/', '/fisheye_rect/')
            
            # 创建绑定正确 cam_name 的回调函数（避免闭包问题）
            def make_callback(name):
                def callback(msg):
                    self.image_callback(msg, name)
                return callback
            
            self.image_subscribers[cam_name] = self.create_subscription(
                Image,
                input_topic,
                make_callback(cam_name),
                10
            )
            
            self.image_publishers[cam_name] = self.create_publisher(
                Image,
                output_topic,
                10
            )
            
            self.get_logger().info(f'Subscribed to {input_topic} -> Publishing to {output_topic}')
        
        # Undistortion maps cache
        self.undistort_maps = {}
        
        self.get_logger().info(f'Omni undistortion initialized (scale={self.scale}, fov_scale={self.fov_scale})')
    
    def load_calibration_file(self, config_file):
        """Load calibration from file"""
        config_paths = []
        
        try:
            pkg_share = get_package_share_directory('seeker')
            config_paths.append(os.path.join(pkg_share, 'config', config_file))
        except:
            pass
        
        src_path = os.path.join(os.path.expanduser('~/ros2_ws/src/seeker1/config'), config_file)
        config_paths.append(src_path)
        config_paths.append(os.path.join(os.path.dirname(__file__), '..', 'config', config_file))
        
        for config_path in config_paths:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    calib_data = yaml.safe_load(f)
                    self.load_calibration(calib_data)
                self.get_logger().info(f'Loaded calibration from: {config_path}')
                return
        
        self.get_logger().error('No calibration file found!')
    
    def load_calibration(self, calib_data):
        """Load calibration parameters from YAML data"""
        for cam_name in ['cam0', 'cam1', 'cam2', 'cam3']:
            if cam_name in calib_data:
                cam_data = calib_data[cam_name]
                intrinsics = cam_data.get('intrinsics', [])
                
                if cam_data.get('camera_model') == 'omni' and len(intrinsics) >= 5:
                    self.camera_params[cam_name] = {
                        'xi': intrinsics[0],
                        'fx': intrinsics[1],
                        'fy': intrinsics[2],
                        'cx': intrinsics[3],
                        'cy': intrinsics[4],
                        'distortion_coeffs': cam_data.get('distortion_coeffs', []),
                        'resolution': cam_data.get('resolution', [1088, 1280])
                    }
                    self.get_logger().info(f'{cam_name}: xi={intrinsics[0]:.3f}, fx={intrinsics[1]:.1f}')
    
    def init_undistort_rectify_map(self, K, D, xi, R, P, size):
        """
        向量化实现的 omnidir.initUndistortRectifyMap（提速）
        """
        # 处理默认参数
        R = np.eye(3) if R is None else R
        if P is None:
            P = np.zeros((3, 4), dtype=np.float32)
            P[:3, :3] = K.copy()
        
        width, height = size
        
        # 提取参数
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        s = K[0, 1]  # 偏斜系数
        
        k1, k2, p1, p2 = D.ravel()[:4]
        
        # 计算逆变换矩阵
        P_square = P[:, :3]
        iKR = np.linalg.inv(P_square)
        
        # 向量化：创建所有像素坐标的网格
        j, i = np.meshgrid(np.arange(width), np.arange(height))
        
        # 计算反投影射线 (齐次坐标) - 向量化
        x = j * iKR[0, 0] + i * iKR[0, 1] + iKR[0, 2]
        y = j * iKR[1, 0] + i * iKR[1, 1] + iKR[1, 2]
        w = j * iKR[2, 0] + i * iKR[2, 1] + iKR[2, 2]
        
        # 归一化到单位球面
        r = np.sqrt(x*x + y*y + w*w)
        Xs = x / r
        Ys = y / r
        Zs = w / r
        
        # 鱼眼投影模型
        denom = Zs + xi
        xu = Xs / denom
        yu = Ys / denom
        
        # 应用畸变模型
        r2 = xu*xu + yu*yu
        r4 = r2*r2
        
        # 径向畸变 + 切向畸变
        xd = (1 + k1*r2 + k2*r4)*xu + 2*p1*xu*yu + p2*(r2 + 2*xu*xu)
        yd = (1 + k1*r2 + k2*r4)*yu + p1*(r2 + 2*yu*yu) + 2*p2*xu*yu
        
        # 转换到像素坐标
        map1 = (fx * xd + s * yd + cx).astype(np.float32)
        map2 = (fy * yd + cy).astype(np.float32)
        
        return map1, map2
    
    def create_omni_undistort_map(self, cam_name, out_height, out_width):
        """
        Create undistortion map for omnidirectional camera
        Uses correct MEI model implementation
        """
        if cam_name not in self.camera_params:
            return None, None
        
        params = self.camera_params[cam_name]
        xi = params['xi']
        fx = params['fx']
        fy = params['fy']
        cx = params['cx']
        cy = params['cy']
        dist_coeffs = np.array(params['distortion_coeffs'][:4], dtype=np.float32)
        
        # 构建相机内参矩阵
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0, 0, 1]], dtype=np.float32)
        
        # 构建投影矩阵 P (控制输出视场和缩放)
        # fov_scale < 1.0: 放大（视野变窄）
        # fov_scale = 1.0: 原始焦距
        # fov_scale > 1.0: 缩小（视野变宽）
        zoom = 1.0 / self.fov_scale
        out_fx = fx * zoom * self.scale  # 同时考虑分辨率缩放
        out_fy = fy * zoom * self.scale
        out_cx = out_width / 2.0
        out_cy = out_height / 2.0
        
        P = np.zeros((3, 4), dtype=np.float32)
        P[:3, :3] = np.array([[out_fx, 0, out_cx],
                              [0, out_fy, out_cy],
                              [0, 0, 1]], dtype=np.float32)
        
        # 使用正确的 omnidir 模型生成映射
        map_x, map_y = self.init_undistort_rectify_map(
            K, dist_coeffs, xi, None, P, (out_width, out_height)
        )
        
        # 生成有效性掩码（map中的坐标必须在原图范围内且为有限数）
        src_h, src_w = params.get('resolution', [1088, 1280])
        src_h = int(src_h)
        src_w = int(src_w)

        valid = np.isfinite(map_x) & np.isfinite(map_y)
        valid &= (map_x >= 0) & (map_x <= (src_w - 1)) & (map_y >= 0) & (map_y <= (src_h - 1))

        return map_x, map_y, valid
    
    def get_undistort_maps(self, cam_name, img_shape):
        """Get or create undistortion maps for a camera"""
        out_height = int(img_shape[0] * self.scale)
        out_width = int(img_shape[1] * self.scale)
        key = f"{cam_name}_{out_height}_{out_width}"
        
        if key in self.undistort_maps:
            return self.undistort_maps[key]
        
        self.get_logger().info(f'Creating undistort maps for {cam_name} ({out_width}x{out_height})...')
        
        result = self.create_omni_undistort_map(cam_name, out_height, out_width)

        if result is not None:
            map_x, map_y, valid = result
            self.undistort_maps[key] = (map_x, map_y, valid)
            self.get_logger().info(f'Maps created for {cam_name}')
            return map_x, map_y, valid

        return None
    
    def image_callback(self, msg, cam_name):
        """Callback for image messages"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Get undistortion maps (may return None or (map_x,map_y,valid))
            map_res = self.get_undistort_maps(cam_name, cv_image.shape[:2])

            if map_res is not None:
                map_x, map_y, valid = map_res

                # Apply undistortion using replicate border to avoid black edges
                undistorted = cv2.remap(cv_image, map_x, map_y, cv2.INTER_CUBIC,
                                       borderMode=cv2.BORDER_REPLICATE)

                # Resize original to output size to fill invalid regions (avoid flicker)
                out_h, out_w = undistorted.shape[:2]
                resized_orig = cv2.resize(cv_image, (out_w, out_h), interpolation=cv2.INTER_LINEAR)

                # Fill invalid pixels from resized original to avoid transient black pixels
                if undistorted.ndim == 3:
                    mask3 = np.repeat(valid[:, :, np.newaxis], undistorted.shape[2], axis=2)
                    undistorted[~mask3] = resized_orig[~mask3]
                else:
                    undistorted[~valid] = resized_orig[~valid]
            else:
                # Fallback to passthrough
                undistorted = cv_image
            
            # Convert back to ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(undistorted, encoding=msg.encoding)
            out_msg.header = msg.header
            
            # Publish
            self.image_publishers[cam_name].publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing {cam_name}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OmniUndistortNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
