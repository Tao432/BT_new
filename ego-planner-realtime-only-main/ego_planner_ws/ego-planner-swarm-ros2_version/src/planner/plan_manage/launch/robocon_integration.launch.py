import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    ROBOCON + ego-planner 集成 Launch 文件
    - 使用 ROBOCON 的点云数据: /lidar_points
    - 使用 ROBOCON 的里程计: /odom
    - 支持手动目标点设置: /move_base_simple/goal
    """
    
    # 定义参数
    drone_id = LaunchConfiguration('drone_id', default=0)
    map_size_x = LaunchConfiguration('map_size_x', default=50.0)
    map_size_y = LaunchConfiguration('map_size_y', default=50.0)
    map_size_z = LaunchConfiguration('map_size_z', default=2.0)
    cloud_topic_name = LaunchConfiguration('cloud_topic_name', default='cloud')
    start_topic_relay = LaunchConfiguration('start_topic_relay', default='true')
    topic_relay_python = LaunchConfiguration('topic_relay_python', default='/usr/bin/python3')
    topic_relay_script = LaunchConfiguration(
        'topic_relay_script',
        default='/home/charlie/pathplan/topic_relay.py')
    start_terrain_filter = LaunchConfiguration('start_terrain_filter', default='false')
    terrain_filter_python = LaunchConfiguration('terrain_filter_python', default='/usr/bin/python3')
    terrain_filter_script = LaunchConfiguration(
        'terrain_filter_script',
        default='/home/charlie/pathplan/terrain_filter.py')
    start_global_scene_map = LaunchConfiguration('start_global_scene_map', default='true')
    global_scene_map_python = LaunchConfiguration('global_scene_map_python', default='/usr/bin/python3')
    global_scene_map_script = LaunchConfiguration(
        'global_scene_map_script',
        default='/home/charlie/pathplan/global_scene_map_publisher.py')
    
    # 声明参数
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id)
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x)
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y)
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z)
    cloud_topic_name_cmd = DeclareLaunchArgument(
        'cloud_topic_name',
        default_value='cloud',
        description='Cloud topic suffix after drone prefix, e.g. cloud or cloud_filtered')
    start_topic_relay_cmd = DeclareLaunchArgument(
        'start_topic_relay',
        default_value='true',
        description='Whether to start topic_relay.py inside this launch file')
    topic_relay_python_cmd = DeclareLaunchArgument(
        'topic_relay_python',
        default_value='/usr/bin/python3',
        description='Python executable used for topic_relay.py')
    topic_relay_script_cmd = DeclareLaunchArgument(
        'topic_relay_script',
        default_value='/home/charlie/pathplan/topic_relay.py',
        description='Absolute path to topic_relay.py')
    start_terrain_filter_cmd = DeclareLaunchArgument(
        'start_terrain_filter',
        default_value='false',
        description='Whether to start terrain_filter.py inside this launch file')
    terrain_filter_python_cmd = DeclareLaunchArgument(
        'terrain_filter_python',
        default_value='/usr/bin/python3',
        description='Python executable used for terrain_filter.py')
    terrain_filter_script_cmd = DeclareLaunchArgument(
        'terrain_filter_script',
        default_value='/home/charlie/pathplan/terrain_filter.py',
        description='Absolute path to terrain_filter.py')
    start_global_scene_map_cmd = DeclareLaunchArgument(
        'start_global_scene_map',
        default_value='true',
        description='Whether to start global_scene_map_publisher.py inside this launch file')
    global_scene_map_python_cmd = DeclareLaunchArgument(
        'global_scene_map_python',
        default_value='/usr/bin/python3',
        description='Python executable used for global_scene_map_publisher.py')
    global_scene_map_script_cmd = DeclareLaunchArgument(
        'global_scene_map_script',
        default_value='/home/charlie/pathplan/global_scene_map_publisher.py',
        description='Absolute path to global_scene_map_publisher.py')
    
    # 包含 advanced_param.launch.py，使用虚拟话题名（稍后重映射）
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            
            # 使用不带前缀的话题名（会被加上 drone_0_ 前缀）
            # 然后在节点启动时重映射到正确的话题
            'odometry_topic': 'odom',
            'cloud_topic': cloud_topic_name,
            
            # 禁用深度相机相关
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            
            # 速度和加速度限制（适配地面机器人）
            'max_vel': str(1.0),
            'max_acc': str(3.0),
            
            # 规划参数
            'planning_horizon': str(3.2),
            'thresh_replan_time': str(0.35),
            'thresh_no_replan_meter': str(0.15),
            'use_odom_for_replan_start': 'True',
            'use_odom_for_finish_check': 'True',
            
            # 使用手动目标点模式
            'flight_type': str(1),
            
            # 虚拟相机参数
            'cx': str(320.0),
            'cy': str(240.0),
            'fx': str(400.0),
            'fy': str(400.0),
            
            # 其他参数
            'use_distinctive_trajs': 'False',
            'obj_num_set': str(0),
        }.items()
    )
    
    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )
    
    # poscmd_2_odom node - 将位置命令转换为 odometry
    poscmd_2_odom_node = Node(
        package='poscmd_2_odom',
        executable='poscmd_2_odom',
        name=['drone_', drone_id, '_poscmd_2_odom'],
        output='screen',
        remappings=[
            ('command', ['drone_', drone_id, '_planning/pos_cmd']),
            ('odometry', '/odometry')  # 输出到 /odometry，供桥接节点使用
        ],
        parameters=[
            {'init_x': -4.9},
            {'init_y': 1.4},
            {'init_z': 0.2}
        ]
    )
    
    # Topic relay node - 将 ROBOCON 话题重映射到 ego-planner 期望的话题
    topic_relay = ExecuteProcess(
        condition=IfCondition(start_topic_relay),
        cmd=[topic_relay_python, topic_relay_script],
        output='screen',
        shell=False
    )

    terrain_filter = ExecuteProcess(
        condition=IfCondition(start_terrain_filter),
        cmd=[terrain_filter_python, terrain_filter_script],
        output='screen',
        shell=False
    )

    global_scene_map = ExecuteProcess(
        condition=IfCondition(start_global_scene_map),
        cmd=[global_scene_map_python, global_scene_map_script],
        output='screen',
        shell=False
    )
    
    # 创建 LaunchDescription
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(drone_id_cmd)
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(cloud_topic_name_cmd)
    ld.add_action(start_topic_relay_cmd)
    ld.add_action(topic_relay_python_cmd)
    ld.add_action(topic_relay_script_cmd)
    ld.add_action(start_terrain_filter_cmd)
    ld.add_action(terrain_filter_python_cmd)
    ld.add_action(terrain_filter_script_cmd)
    ld.add_action(start_global_scene_map_cmd)
    ld.add_action(global_scene_map_python_cmd)
    ld.add_action(global_scene_map_script_cmd)
    
    # 添加话题中继节点（必须在 ego-planner 之前启动）
    ld.add_action(topic_relay)
    ld.add_action(global_scene_map)
    ld.add_action(terrain_filter)
    
    # 添加 ego-planner 节点
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(poscmd_2_odom_node)
    
    return ld
