#!/usr/bin/env python3
"""
Launch file to start seeker node with rqt showing all camera images
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_image_transport_arg = DeclareLaunchArgument(
        'use_image_transport',
        default_value='false',
        description='是否使用image_transport传输图像'
    )
    
    pub_disparity_img_arg = DeclareLaunchArgument(
        'pub_disparity_img',
        default_value='true',
        description='是否发布视差图像'
    )
    
    pub_disparity_arg = DeclareLaunchArgument(
        'pub_disparity',
        default_value='true',
        description='是否发布视差消息'
    )
    
    pub_imu_arg = DeclareLaunchArgument(
        'pub_imu',
        default_value='true',
        description='是否发布IMU数据'
    )
    
    time_sync_arg = DeclareLaunchArgument(
        'time_sync',
        default_value='true',
        description='是否启用时间同步'
    )
    
    use_undistort_arg = DeclareLaunchArgument(
        'use_undistort',
        default_value='false',
        description='是否启用图像去畸变'
    )
    
    undistort_scale_arg = DeclareLaunchArgument(
        'undistort_scale',
        default_value='0.5',
        description='去畸变图像缩放比例'
    )
    
    undistort_alpha_arg = DeclareLaunchArgument(
        'undistort_alpha',
        default_value='0.0',
        description='去畸变alpha参数 (0.0=裁剪黑边, 1.0=保留所有像素)'
    )
    
    undistort_fov_scale_arg = DeclareLaunchArgument(
        'undistort_fov_scale',
        default_value='0.4',
        description='视场缩放 (0.3-0.5: 放大中心区域避免圆形边缘)'
    )

    seeker_node = Node(
        package='seeker',
        executable='seeker_node',
        name='seeker_node',
        output='screen',
        parameters=[{
            'use_image_transport': LaunchConfiguration('use_image_transport'),
            'pub_disparity_img': LaunchConfiguration('pub_disparity_img'),
            'pub_disparity': LaunchConfiguration('pub_disparity'),
            'pub_imu': LaunchConfiguration('pub_imu'),
            'time_sync': LaunchConfiguration('time_sync'),
            'imu_link': 'imu',
        }]
    )
    
    # Undistortion node
    undistort_node = Node(
        package='seeker',
        executable='undistort_node.py',
        name='undistort_node',
        output='screen',
        parameters=[{
            'config_file': 'seeker_omni_depth/kalibr_cam_chain.yaml',
            'scale': LaunchConfiguration('undistort_scale'),
            'alpha': LaunchConfiguration('undistort_alpha'),
            'fov_scale': LaunchConfiguration('undistort_fov_scale')
        }],
        condition=IfCondition(LaunchConfiguration('use_undistort'))
    )
    
    # Get perspective file path
    try:
        pkg_share = get_package_share_directory('seeker')
        perspective_file = os.path.join(pkg_share, 'config', 'seeker_images.perspective')
    except:
        # Fallback to source directory
        perspective_file = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'seeker_images.perspective')
    
    # Launch rqt with the perspective file to show all images in one window
    rqt_process = ExecuteProcess(
        cmd=['rqt', '--perspective-file', perspective_file, '--force-discover'],
        output='screen'
    )

    return LaunchDescription([
        use_image_transport_arg,
        pub_disparity_img_arg,
        pub_disparity_arg,
        pub_imu_arg,
        time_sync_arg,
        use_undistort_arg,
        undistort_scale_arg,
        undistort_alpha_arg,
        undistort_fov_scale_arg,
        seeker_node,
        undistort_node,
        rqt_process
    ])
