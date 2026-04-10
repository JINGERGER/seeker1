from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
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

    pub_fisheye_raw_arg = DeclareLaunchArgument(
        'pub_fisheye_raw',
        default_value='false',
        description='是否发布原始鱼眼图像 /fisheye/*/image_raw 和 /all/compressed（默认关，去畸变节点会订阅）'
    )
    
    pub_disparity_img_arg = DeclareLaunchArgument(
        'pub_disparity_img',
        default_value='false',
        description='是否发布视差原始图像 /*/disparity/image_raw'
    )
    
    pub_disparity_arg = DeclareLaunchArgument(
        'pub_disparity',
        default_value='false',
        description='是否发布视差消息 /*/disparity'
    )
    
    pub_imu_arg = DeclareLaunchArgument(
        'pub_imu',
        default_value='false',
        description='是否发布IMU数据 /imu_data_raw'
    )
    
    time_sync_arg = DeclareLaunchArgument(
        'time_sync',
        default_value='true',
        description='是否启用时间同步'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='是否启动RViz2'
    )
    
    use_undistort_arg = DeclareLaunchArgument(
        'use_undistort',
        default_value='true',
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
        default_value='1.5',
        description='视场缩放 (>1.0: 视野变宽, <1.0: 视野变窄, 推荐: 1.2-2.0)'
    )
    
    undistort_jpeg_quality_arg = DeclareLaunchArgument(
        'undistort_jpeg_quality',
        default_value='80',
        description='去畸变压缩图像JPEG质量 (1-100)'
    )

    undistort_pub_raw_arg = DeclareLaunchArgument(
        'undistort_pub_raw',
        default_value='false',
        description='是否同时发布未压缩原始图像（默认关闭以节省带宽）'
    )
    
    use_depth_arg = DeclareLaunchArgument(
        'use_depth',
        default_value='false',
        description='是否启用深度图转换'
    )

    seeker_node = Node(
        package='seeker',
        executable='seeker_node',
        name='seeker_node',
        output='screen',
        parameters=[{
            'use_image_transport': LaunchConfiguration('use_image_transport'),
            'pub_fisheye_raw': LaunchConfiguration('pub_fisheye_raw'),
            'pub_disparity_img': LaunchConfiguration('pub_disparity_img'),
            'pub_disparity': LaunchConfiguration('pub_disparity'),
            'pub_imu': LaunchConfiguration('pub_imu'),
            'time_sync': LaunchConfiguration('time_sync'),
            'imu_link': 'imu',
        }]
    )
    
    # RViz2配置文件路径
    rviz_config_dir = os.path.join(
        get_package_share_directory('seeker'),
        'config',
        'seeker.rviz'
    )
    
    # Omni Undistortion node (correct MEI model implementation)
    undistort_node = Node(
        package='seeker',
        executable='omni_undistort_node.py',
        name='omni_undistort_node',
        output='screen',
        parameters=[{
            'config_file': 'seeker_omni_depth/kalibr_cam_chain.yaml',
            'scale': LaunchConfiguration('undistort_scale'),
            'fov_scale': LaunchConfiguration('undistort_fov_scale'),
            'jpeg_quality': LaunchConfiguration('undistort_jpeg_quality'),
            'pub_raw': LaunchConfiguration('undistort_pub_raw'),
        }],
        condition=IfCondition(LaunchConfiguration('use_undistort'))
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Depth conversion node (always started, internal 'enabled' parameter controls publishing)
    depth_node = Node(
        package='seeker',
        executable='disparity_to_depth.py',
        name='disparity_to_depth',
        output='screen',
        parameters=[{
            'enabled': LaunchConfiguration('use_depth')
        }]
    )

    return LaunchDescription([
        use_image_transport_arg,
        pub_fisheye_raw_arg,
        pub_disparity_img_arg,
        pub_disparity_arg,
        pub_imu_arg,
        time_sync_arg,
        use_rviz_arg,
        use_undistort_arg,
        undistort_scale_arg,
        undistort_alpha_arg,
        undistort_fov_scale_arg,
        undistort_jpeg_quality_arg,
        undistort_pub_raw_arg,
        use_depth_arg,
        seeker_node,
        undistort_node,
        depth_node,
        rviz_node
    ])
