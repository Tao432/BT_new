from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REPO_ROOT = Path(__file__).resolve().parent


def generate_launch_description():
    drone_id = LaunchConfiguration("drone_id", default=0)
    map_size_x = LaunchConfiguration("map_size_x", default=50.0)
    map_size_y = LaunchConfiguration("map_size_y", default=50.0)
    map_size_z = LaunchConfiguration("map_size_z", default=2.0)
    cloud_topic_name = LaunchConfiguration("cloud_topic_name", default="cloud_filtered")

    start_topic_relay = LaunchConfiguration("start_topic_relay", default="false")
    topic_relay_python = LaunchConfiguration("topic_relay_python", default="/usr/bin/python3")
    topic_relay_script = LaunchConfiguration(
        "topic_relay_script", default=str(REPO_ROOT / "topic_relay.py")
    )

    start_terrain_filter = LaunchConfiguration("start_terrain_filter", default="true")
    terrain_filter_python = LaunchConfiguration(
        "terrain_filter_python", default="/usr/bin/python3"
    )
    terrain_filter_script = LaunchConfiguration(
        "terrain_filter_script", default=str(REPO_ROOT / "terrain_filter.py")
    )

    declared_args = [
        DeclareLaunchArgument("drone_id", default_value=drone_id),
        DeclareLaunchArgument("map_size_x", default_value=map_size_x),
        DeclareLaunchArgument("map_size_y", default_value=map_size_y),
        DeclareLaunchArgument("map_size_z", default_value=map_size_z),
        DeclareLaunchArgument(
            "cloud_topic_name",
            default_value="cloud_filtered",
            description="Cloud topic suffix after drone prefix",
        ),
        DeclareLaunchArgument(
            "start_topic_relay",
            default_value="false",
            description="Whether to start topic_relay.py inside this launch file",
        ),
        DeclareLaunchArgument(
            "topic_relay_python",
            default_value="/usr/bin/python3",
            description="Python executable used for topic_relay.py",
        ),
        DeclareLaunchArgument(
            "topic_relay_script",
            default_value=str(REPO_ROOT / "topic_relay.py"),
            description="Path to topic_relay.py",
        ),
        DeclareLaunchArgument(
            "start_terrain_filter",
            default_value="true",
            description="Whether to start terrain_filter.py inside this launch file",
        ),
        DeclareLaunchArgument(
            "terrain_filter_python",
            default_value="/usr/bin/python3",
            description="Python executable used for terrain_filter.py",
        ),
        DeclareLaunchArgument(
            "terrain_filter_script",
            default_value=str(REPO_ROOT / "terrain_filter.py"),
            description="Path to terrain_filter.py",
        ),
    ]

    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("ego_planner")) / "launch" / "advanced_param.launch.py")
        ),
        launch_arguments={
            "drone_id": drone_id,
            "map_size_x_": map_size_x,
            "map_size_y_": map_size_y,
            "map_size_z_": map_size_z,
            "odometry_topic": "odom",
            "cloud_topic": cloud_topic_name,
            "camera_pose_topic": "pcl_render_node/camera_pose",
            "depth_topic": "pcl_render_node/depth",
            "max_vel": str(1.0),
            "max_acc": str(3.0),
            "planning_horizon": str(3.2),
            "thresh_replan_time": str(0.35),
            "thresh_no_replan_meter": str(0.15),
            "use_odom_for_replan_start": "True",
            "use_odom_for_finish_check": "True",
            "obstacles_inflation": str(0.09),
            "flight_type": str(1),
            "cx": str(320.0),
            "cy": str(240.0),
            "fx": str(400.0),
            "fy": str(400.0),
            "use_distinctive_trajs": "False",
            "obj_num_set": str(0),
        }.items(),
    )

    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        name=["drone_", drone_id, "_traj_server"],
        output="screen",
        remappings=[
            ("position_cmd", ["drone_", drone_id, "_planning/pos_cmd"]),
            ("planning/bspline", ["drone_", drone_id, "_planning/bspline"]),
        ],
        parameters=[{"traj_server/time_forward": 1.0}],
    )

    poscmd_2_odom_node = Node(
        package="poscmd_2_odom",
        executable="poscmd_2_odom",
        name=["drone_", drone_id, "_poscmd_2_odom"],
        output="screen",
        remappings=[
            ("command", ["drone_", drone_id, "_planning/pos_cmd"]),
            ("odometry", "/odometry"),
        ],
        parameters=[
            {"init_x": -4.9},
            {"init_y": 1.4},
            {"init_z": 0.2},
        ],
    )

    topic_relay = ExecuteProcess(
        condition=IfCondition(start_topic_relay),
        cmd=[topic_relay_python, topic_relay_script],
        output="screen",
        shell=False,
    )

    terrain_filter = ExecuteProcess(
        condition=IfCondition(start_terrain_filter),
        cmd=[terrain_filter_python, terrain_filter_script],
        output="screen",
        shell=False,
    )

    ld = LaunchDescription()
    for action in declared_args:
        ld.add_action(action)

    ld.add_action(topic_relay)
    ld.add_action(terrain_filter)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(poscmd_2_odom_node)
    return ld
