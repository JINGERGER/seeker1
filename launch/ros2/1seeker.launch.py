from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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

    return LaunchDescription([
        use_image_transport_arg,
        pub_disparity_img_arg,
        pub_disparity_arg,
        pub_imu_arg,
        time_sync_arg,
        seeker_node
    ])
