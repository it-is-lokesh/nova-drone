from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# gz topic -t /nova/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[400, 400, 400, 400]'

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nova_main = get_package_share_directory('nova_main')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-s -r -z  1000000 nova.sdf --iterations 1000000'
        }.items(),
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=os.path.join(pkg_nova_main, 'config', 'nova.yaml')
    )

    init_imu_interface = Node(
        package='nova_interface',
        executable='node_subscriber_imu',
        name='node_subscriber_imu',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
    )

    init_altimeter_interface = Node(
        package='nova_interface',
        executable='node_subscriber_altimeter',
        name='node_subscriber_altimeter',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
    )

    init_navsat_interface = Node(
        package='nova_interface',
        executable='node_subscriber_navsat',
        name='node_subscriber_navsat',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
    )

    init_shm = Node(
        package='nova_application',
        executable='nv_shm_init',
        name='nv_shm_init',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        ros_gz_bridge,
        init_shm,
        init_imu_interface,
        init_altimeter_interface,
        init_navsat_interface,
    ])