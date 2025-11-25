#!/usr/bin/env python3
"""
Launch file to start seeker with point cloud generation
"""

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Seeker node arguments
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
    
    use_pointcloud_arg = DeclareLaunchArgument(
        'use_pointcloud',
        default_value='true',
        description='是否生成点云'
    )

    # Seeker node
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
    
    # Point Cloud nodes for each direction
    # Note: We need camera_info topics which may not be published yet
    # For now, we'll use disparity_to_depth conversion
    
    # Front point cloud
    front_pointcloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='front_pointcloud',
        remappings=[
            ('image_rect', '/front/disparity/image_raw'),
            ('camera_info', '/front/camera_info'),
            ('points', '/front/points')
        ],
        condition=IfCondition(LaunchConfiguration('use_pointcloud'))
    )
    
    # Right point cloud
    right_pointcloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='right_pointcloud',
        remappings=[
            ('image_rect', '/right/disparity/image_raw'),
            ('camera_info', '/right/camera_info'),
            ('points', '/right/points')
        ],
        condition=IfCondition(LaunchConfiguration('use_pointcloud'))
    )
    
    # Back point cloud
    back_pointcloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='back_pointcloud',
        remappings=[
            ('image_rect', '/back/disparity/image_raw'),
            ('camera_info', '/back/camera_info'),
            ('points', '/back/points')
        ],
        condition=IfCondition(LaunchConfiguration('use_pointcloud'))
    )
    
    # Left point cloud
    left_pointcloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='left_pointcloud',
        remappings=[
            ('image_rect', '/left/disparity/image_raw'),
            ('camera_info', '/left/camera_info'),
            ('points', '/left/points')
        ],
        condition=IfCondition(LaunchConfiguration('use_pointcloud'))
    )

    return LaunchDescription([
        use_image_transport_arg,
        pub_disparity_img_arg,
        pub_disparity_arg,
        pub_imu_arg,
        time_sync_arg,
        use_pointcloud_arg,
        seeker_node,
        front_pointcloud_node,
        right_pointcloud_node,
        back_pointcloud_node,
        left_pointcloud_node,
    ])
