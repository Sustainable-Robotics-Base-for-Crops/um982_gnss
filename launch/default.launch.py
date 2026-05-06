import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_dir = FindPackageShare('um982_gnss')
    yaml_path = PathJoinSubstitution([package_dir, 'config', 'default.yaml'])
    ntrip_client_yaml_path = PathJoinSubstitution(
        [package_dir, 'config', 'ntrip_client.yaml'])

    container = ComposableNodeContainer(
        name='um982_gnss_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='um982_gnss',
                plugin='um982_gnss::UM982Gnss',
                name='um982_gnss',
                parameters=[yaml_path]),
            ComposableNode(
                package='um982_gnss',
                plugin='um982_gnss::NtripClient',
                name='ntrip_client',
                parameters=[ntrip_client_yaml_path]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager',
                parameters=[{'autostart': True,
                             'node_names': ['/um982_gnss', '/ntrip_client']}])
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
