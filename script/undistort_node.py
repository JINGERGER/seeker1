#!/usr/bin/env python3
"""
Simple image undistortion node for ROS2
Subscribes to fisheye images and publishes undistorted versions
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


class UndistortNode(Node):
    def __init__(self):
        super().__init__('undistort_node')
        
        # Declare parameters
        self.declare_parameter('config_file', 'seeker_omni_depth/kalibr_cam_chain.yaml')
        self.declare_parameter('scale', 0.5)
        self.declare_parameter('alpha', 0.0)  # 0=crop all invalid pixels, 1=keep all pixels
        self.declare_parameter('fov_scale', 0.5)  # For omni cameras: how much of FOV to use (0.5 = central 50%)
        
        # Get parameters
        config_file = self.get_parameter('config_file').value
        self.scale = self.get_parameter('scale').value
        self.alpha = self.get_parameter('alpha').value
        self.fov_scale = self.get_parameter('fov_scale').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load calibration data
        self.camera_params = {}
        calibration_loaded = False
        
        try:
            # Try multiple paths for calibration file
            config_paths = []
            
            # 1. Try installed share directory
            try:
                pkg_share = get_package_share_directory('seeker')
                config_paths.append(os.path.join(pkg_share, 'config', config_file))
            except:
                pass
            
            # 2. Try source workspace
            src_path = os.path.join(os.path.expanduser('~/ros2_ws/src/seeker1/config'), config_file)
            config_paths.append(src_path)
            
            # 3. Try relative to script
            config_paths.append(os.path.join(os.path.dirname(__file__), '..', 'config', config_file))
            
            for config_path in config_paths:
                self.get_logger().info(f'Trying calibration path: {config_path}')
                if os.path.exists(config_path):
                    with open(config_path, 'r') as f:
                        calib_data = yaml.safe_load(f)
                        self.load_calibration(calib_data)
                    self.get_logger().info(f'Successfully loaded calibration from: {config_path}')
                    calibration_loaded = True
                    break
            
            if not calibration_loaded:
                self.get_logger().warn(f'Calibration file not found in any of the searched paths')
                self.get_logger().warn('Searched paths:')
                for path in config_paths:
                    self.get_logger().warn(f'  - {path}')
                self.get_logger().warn('Will use passthrough mode (no undistortion)')
                self.get_logger().warn('To generate calibration file, run: python3 script/1get_kalibr_info.py')
        except Exception as e:
            self.get_logger().error(f'Error loading calibration: {e}')
            self.get_logger().warn('Will use passthrough mode (no undistortion)')
        
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
            
            self.image_subscribers[cam_name] = self.create_subscription(
                Image,
                input_topic,
                lambda msg, name=cam_name: self.image_callback(msg, name),
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
    
    def load_calibration(self, calib_data):
        """Load calibration parameters from YAML data"""
        for cam_name in ['cam0', 'cam1', 'cam2', 'cam3']:
            if cam_name in calib_data:
                cam_data = calib_data[cam_name]
                
                # Extract intrinsics
                intrinsics = cam_data.get('intrinsics', [])
                distortion_model = cam_data.get('distortion_model', 'radtan')
                distortion_coeffs = cam_data.get('distortion_coeffs', [])
                resolution = cam_data.get('resolution', [640, 480])
                
                if len(intrinsics) >= 4:
                    # Store camera parameters
                    self.camera_params[cam_name] = {
                        'intrinsics': intrinsics,
                        'camera_model': cam_data.get('camera_model', 'pinhole'),
                        'distortion_model': distortion_model,
                        'distortion_coeffs': distortion_coeffs,
                        'resolution': resolution
                    }
                    self.get_logger().info(f'Loaded calibration for {cam_name} (model: {cam_data.get("camera_model", "pinhole")})')
    
    def get_undistort_maps(self, cam_name, img_shape):
        """Get or create undistortion maps for a camera"""
        key = f"{cam_name}_{img_shape[0]}_{img_shape[1]}"
        
        if key in self.undistort_maps:
            return self.undistort_maps[key]
        
        if cam_name not in self.camera_params:
            return None, None
        
        params = self.camera_params[cam_name]
        intrinsics = params['intrinsics']
        
        # Check camera model
        camera_model = params.get('camera_model', 'pinhole')
        
        # Handle omni/omnidirectional model (MEI model with xi parameter)
        if camera_model == 'omni' and len(intrinsics) >= 5:
            # Omni model: [xi, fx, fy, cx, cy]
            xi, fx, fy, cx, cy = intrinsics[:5]
            
            # For omni cameras, we use fisheye undistortion
            K = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float32)
            
            D = np.array(params['distortion_coeffs'][:4], dtype=np.float32)
            
            # For omni/fisheye cameras, we need to create a rectilinear projection
            # that zooms into the center to avoid the circular boundary
            try:
                # Create a new camera matrix with reduced FOV (zoomed in to avoid circle edges)
                # Increase focal length to "zoom in" and crop to rectangular view
                zoom_factor = 1.0 / self.fov_scale  # Higher zoom = smaller FOV = less circular boundary
                
                new_K = K.copy()
                new_K[0, 0] *= zoom_factor * self.scale  # fx with zoom and scale
                new_K[1, 1] *= zoom_factor * self.scale  # fy with zoom and scale
                new_K[0, 2] = img_shape[1] * self.scale / 2  # cx centered
                new_K[1, 2] = img_shape[0] * self.scale / 2  # cy centered
                
                new_size = (int(img_shape[1] * self.scale), int(img_shape[0] * self.scale))
                
                map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                    K, D, np.eye(3), new_K, new_size, cv2.CV_16SC2
                )
            except Exception as e:
                self.get_logger().warn(f'Fisheye undistortion failed for {cam_name}: {e}, using radtan')
                # Fallback to radtan model
                D = np.array(params['distortion_coeffs'], dtype=np.float32)
                new_K, roi = cv2.getOptimalNewCameraMatrix(
                    K, D, img_shape[::-1], 0.0,  # alpha=0 to crop out invalid pixels
                    (int(img_shape[1] * self.scale), int(img_shape[0] * self.scale))
                )
                
                new_size = (int(img_shape[1] * self.scale), int(img_shape[0] * self.scale))
                map1, map2 = cv2.initUndistortRectifyMap(
                    K, D, np.eye(3), new_K, new_size, cv2.CV_16SC2
                )
        else:
            # Standard pinhole model
            fx, fy, cx, cy = intrinsics[:4] if len(intrinsics) >= 4 else intrinsics
            K = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float32)
            
            D = np.array(params['distortion_coeffs'], dtype=np.float32)
            
            # Handle different distortion models
            distortion_model = params['distortion_model']
            
            if distortion_model == 'equidistant' or distortion_model == 'fisheye':
                # Use fisheye model
                new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    K, D[:4], img_shape[::-1], np.eye(3), balance=self.alpha
                )
                
                new_K[0, 0] *= self.scale
                new_K[1, 1] *= self.scale
                new_K[0, 2] = img_shape[1] * self.scale / 2
                new_K[1, 2] = img_shape[0] * self.scale / 2
                
                new_size = (int(img_shape[1] * self.scale), int(img_shape[0] * self.scale))
                
                map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                    K, D[:4], np.eye(3), new_K, new_size, cv2.CV_16SC2
                )
            else:
                # Use standard radial-tangential model
                new_K = cv2.getOptimalNewCameraMatrix(
                    K, D, img_shape[::-1], self.alpha,
                    (int(img_shape[1] * self.scale), int(img_shape[0] * self.scale))
                )[0]
                
                new_size = (int(img_shape[1] * self.scale), int(img_shape[0] * self.scale))
                
                map1, map2 = cv2.initUndistortRectifyMap(
                    K, D, np.eye(3), new_K, new_size, cv2.CV_16SC2
                )
        
        # Cache the maps
        self.undistort_maps[key] = (map1, map2)
        self.get_logger().info(f'Created undistortion maps for {cam_name} with shape {img_shape}')
        
        return map1, map2
    
    def image_callback(self, msg, cam_name):
        """Callback for image messages"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Get undistortion maps
            map1, map2 = self.get_undistort_maps(cam_name, cv_image.shape[:2])
            
            if map1 is not None and map2 is not None:
                # Apply undistortion
                undistorted = cv2.remap(cv_image, map1, map2, cv2.INTER_LINEAR)
            else:
                # Passthrough if no calibration
                undistorted = cv_image
            
            # Convert back to ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(undistorted, encoding=msg.encoding)
            out_msg.header = msg.header
            
            # Publish
            self.image_publishers[cam_name].publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image from {cam_name}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UndistortNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
