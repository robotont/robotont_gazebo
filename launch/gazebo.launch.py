from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robotont_gazebo_pkg = get_package_share_directory('robotont_gazebo')

    world_arg = DeclareLaunchArgument('world', default_value='worlds/empty.world')
    world_path = PathJoinSubstitution([FindPackageShare('robotont_gazebo'), 'worlds', LaunchConfiguration('world')])

    x_pos_arg = DeclareLaunchArgument('x', default_value='0')
    y_pos_arg = DeclareLaunchArgument('y', default_value='0')
    z_pos_arg = DeclareLaunchArgument('z', default_value='0')

    model_arg = DeclareLaunchArgument('model', default_value='robotont_gazebo_nuc')
    generation_arg = DeclareLaunchArgument('generation', default_value='3')

    urdf_file = PathJoinSubstitution([
        robotont_gazebo_pkg,
        'urdf',
        PythonExpression(["'", LaunchConfiguration('model'), ".urdf.xacro'"])
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", urdf_file,
        " generation:=", LaunchConfiguration('generation')
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    gazebo_model_path = SetEnvironmentVariable(
        name='GZ_SIM_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            ':', PathJoinSubstitution([FindPackageShare('robotont_description'), 'meshes']),
            ':', PathJoinSubstitution([FindPackageShare('robotont_nuc_description'), 'meshes']),
            ':', PathJoinSubstitution([FindPackageShare('robotont_gazebo')])
        ]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value=''),
            ':', FindPackageShare('robotont_description'),
            ':', FindPackageShare('robotont_nuc_description'),
            ':', PathJoinSubstitution([FindPackageShare('robotont_gazebo')])
        ]
    )

    gazebo_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', world_path],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    spawn_urdf_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_urdf',
        output='screen',
        arguments=[
            '-name', 'robotont',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-string', robot_description_content
        ]
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        condition=IfCondition(PythonExpression(["\"", LaunchConfiguration('model'), "\" == \"robotont_gazebo_nuc\""])),
        arguments=[
            '/world/default/model/robotont/link/base_footprint/sensor/rs_d435i/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/default/model/robotont/link/base_footprint/sensor/rs_d435i/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/default/model/robotont/link/base_footprint/sensor/rs_d435i/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/world/default/model/robotont/link/base_footprint/sensor/rs_d435i/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        remappings=[
            ('/world/default/model/robotont/link/base_footprint/sensor/rs_d435i/image', '/camera/color/image_raw'),
            ('/world/default/model/robotont/link/base_footprint/sensor/rs_d435i/depth_image', '/camera/depth/image_raw'),
            ('/world/default/model/robotont/link/base_footprint/sensor/rs_d435i/points', '/camera/depth/points'),
            ('/world/default/model/robotont/link/base_footprint/sensor/rs_d435i/camera_info', '/camera/color/camera_info')
        ]
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        condition=IfCondition(PythonExpression(["\"", LaunchConfiguration('model'), "\" == \"robotont_gazebo_lidar\""])),
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        remappings=[
            ('/lidar', '/scan'),
        ]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/world/default/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        remappings=[('/world/default/clock', '/clock')]
    )

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        output='screen',
        arguments=['/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model']
    )

    fake_driver_node = Node(
        package='robotont_driver',
        executable='fake_driver_node',
        name='driver',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_arg,
        x_pos_arg,
        y_pos_arg,
        z_pos_arg,
        model_arg,
        generation_arg,
        gazebo_model_path,
        gazebo_resource_path,
        gazebo_sim,
        robot_state_publisher_node,
        spawn_urdf_node,
        camera_bridge,
        lidar_bridge,
        clock_bridge,
        joint_state_bridge,
        fake_driver_node
    ])
