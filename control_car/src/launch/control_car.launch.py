from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的路径
    package_dir = get_package_share_directory('control_car')
    
    # 参数文件路径
    params_file = os.path.join(package_dir, 'config', 'car_params.yaml')
    
    # 声明启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB_A',
        description='串口设备路径'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='是否启用调试模式'
    )
    
    # 创建节点
    control_car_node = Node(
        package='control_car',
        executable='control_car_node',
        name='control_car_node',
        parameters=[
            params_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'debug_mode': LaunchConfiguration('debug_mode')
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        debug_mode_arg,
        control_car_node
    ])
