import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 直接使用源码目录的配置文件（无需重新编译即可生效）
    # 获取当前 launch 文件所在目录
    import sys
    import os
    
    # 方法1：使用环境变量或默认路径
    # 优先使用源码目录，如果不存在则回退到 install 目录
    source_config = '/home/sunrise/RacingDev/dev_ws/src/racing/racing_inertial_nav/config/params.yaml'
    
    if os.path.exists(source_config):
        config_file = source_config
        print(f"✅ Using source config: {config_file}")
    else:
        # 回退到 install 目录
        pkg_dir = get_package_share_directory('racing_inertial_nav')
        config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
        print(f"⚠️  Using install config: {config_file}")
    
    # Create inertial navigation controller node
    inertial_nav_node = Node(
        package='racing_inertial_nav',
        executable='inertial_nav_controller',
        name='inertial_nav_controller',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/odom_combined', '/odom_combined'),  # EKF fused odometry
            ('/cmd_vel', '/cmd_vel'),              # Velocity command
        ]
    )
    
    return LaunchDescription([
        inertial_nav_node
    ])
