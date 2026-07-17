import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav_bt_dir = get_package_share_directory('nav_behaviour_tree')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    bringup_config_path = os.path.join(nav_bt_dir, 'config', 'bringup.yaml')
    with open(bringup_config_path) as f:
        bringup_config = yaml.safe_load(f) or {}

    default_map = bringup_config.get('map', '')
    if default_map and not os.path.isabs(default_map):
        default_map = os.path.join(nav_bt_dir, 'maps', default_map)
    default_use_sim_time = str(bringup_config.get('use_sim_time', 'true')).lower()

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    tree_id = LaunchConfiguration('tree_id')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value=default_use_sim_time,
        description='Use simulation clock if true')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map yaml file to load')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_bt_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 params file')
    declare_tree_id = DeclareLaunchArgument(
        'tree_id', default_value='MainTree',
        description='Which BT to run; see bt_xml/main_tree.xml for the available IDs')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to launch rviz2 with the bundled config')

    # Brings up AMCL, controller/planner/behavior servers, bt_navigator,
    # waypoint_follower, velocity_smoother and the lifecycle manager -- all
    # configured via params_file (see config/nav2_params.yaml)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    bt_main = Node(
        package='nav_behaviour_tree',
        executable='bt_main',
        name='nav_behaviour_tree',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'tree_id': tree_id}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(nav_bt_dir, 'rviz', 'nav_behaviour_tree.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # PointCloud2 (/livox/lidar) -> LaserScan (/scan) for slam_toolbox to consume.
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            os.path.join(nav_bt_dir, 'config', 'pointcloud_to_laserscan.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
    )

    # hdas_msg/Imu (/hdas/imu_chassis) -> sensor_msgs/Imu (/imu), for the EKF below.
    imu_hdas_translator = Node(
        package='nav_behaviour_tree',
        executable='imu_hdas_translator.py',
        name='imu_hdas_translator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # fuses /odom (wheel FK, from robocup_nav) + /imu (above) into odom -> base_link.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(nav_bt_dir, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_tree_id,
        declare_use_rviz,
        nav2_bringup,
        bt_main,
        rviz_node,
        pointcloud_to_laserscan,
        imu_hdas_translator,
        ekf_node,
    ])
