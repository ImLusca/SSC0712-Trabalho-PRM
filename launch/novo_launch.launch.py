from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch.actions import SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

def generate_launch_description():
    # --- Variáveis Comuns ---
    # Definição de 'use_sim_time' para todos os nós que precisam usar o tempo simulado
    use_sim_time = True

    # ------------------------------------------------------
    # Configuração de variáveis de ambiente para o Gazebo
    # ------------------------------------------------------
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
            os.environ.get('LD_LIBRARY_PATH', default='')
        ])
    }
    gz_verbosity = '3'

    # ------------------------------------------------------
    # Caminho para o mundo a ser carregado no Gazebo
    # ------------------------------------------------------
    pkg_share = FindPackageShare("prm").find("prm")
    world_file_name = 'arena.sdf'
    world_path = PathJoinSubstitution([
        pkg_share,
        "world",
        world_file_name
    ])

    # ------------------------------------------------------
    # Inicialização do simulador Gazebo
    # ------------------------------------------------------
    gazebo = ExecuteProcess(
        cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-v', gz_verbosity, world_path],
        output='screen',
        additional_env=gz_env,
        shell=False,
    )

    # ------------------------------------------------------
    # Configuração do caminho de recursos do Gazebo
    # ------------------------------------------------------
    gz_models_path = ":".join([
        pkg_share,
        os.path.join(pkg_share, "models")
    ])

    gz_set_env = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=gz_models_path,
    )

    # ------------------------------------------------------
    # Ponte Gazebo <-> ROS 2 (para o mundo e a câmera do céu)
    # ------------------------------------------------------
    world_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_world",
        arguments=[
            "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image"
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # ------------------------------------------------------
    # Caminho para o arquivo Xacro do robô e processamento
    # ------------------------------------------------------
    urdf_path = PathJoinSubstitution([
        FindPackageShare("prm"),
        "description",
        "robot.urdf.xacro"
    ])
    robot_urdf_final = Command(["xacro ", urdf_path])

    # ------------------------------------------------------
    # Nodo robot_state_publisher
    # ------------------------------------------------------
    diff_drive_params = PathJoinSubstitution([
        FindPackageShare("prm"),
        "config",
        "diff_drive_controller_velocity.yaml"
    ])

    slam_params = PathJoinSubstitution([
        FindPackageShare("prm"),
        "config",
        "mapper_params_online_async.yaml"
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_urdf_final, "use_sim_time": use_sim_time}
        ],
    )

    # ------------------------------------------------------
    # Preparação do sistema de controle das rodas do robô
    # ------------------------------------------------------
    load_joint_state_controller = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        shell=False,
        output="screen",
    )

    start_diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_diff_drive_base_controller",
        arguments=["diff_drive_base_controller"],
        parameters=[diff_drive_params],
        output="screen",
    )

    relay_odom = Node(
        name="relay_odom",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/diff_drive_base_controller/odom",
                "output_topic": "/odom",
                "use_sim_time": use_sim_time
            }
        ],
        output="screen",
    )

    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diff_drive_base_controller/cmd_vel_unstamped",
                "use_sim_time": use_sim_time
            }
        ],
        output="screen",
    )

    # ------------------------------------------------------
    # RViz: visualização do robô
    # ------------------------------------------------------
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("prm"),
        "rviz",
        "rviz_config.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ------------------------------------------------------
    # Spawn do robô no simulador Gazebo
    # ------------------------------------------------------
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "prm_robot",
            "-topic", "robot_description",
            "-z", "1.0",
            "-x", "-2.0",
            "--ros-args", "--log-level", "warn"
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ------------------------------------------------------
    # Ponte Gazebo <-> ROS 2 (para o robô e seus sensores)
    # ------------------------------------------------------
    robot_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_prm_robot",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
            "/robot_cam/labels_map@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/robot_cam/colored_map@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/robot_cam/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/model/prm_robot/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # ------------------------------------------------------
    # Nodos personalizados e SLAM
    # ------------------------------------------------------
    odom_gt = Node(
        package="prm",
        executable="ground_truth_odometry",
        name="odom_gt",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robo_mapper = Node(
        package="prm",
        executable="robo_mapper",
        name="robo_mapper",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    slammer = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )

    # ------------------------------------------------------
    # Definição da descrição completa do lançamento
    # ------------------------------------------------------
    return LaunchDescription([
        gz_set_env,
        gazebo,
        world_bridge, # Ponte para o mundo (câmera do céu)
        robot_bridge, # Ponte para o robô e seus sensores
        robot_state_publisher_node,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[start_diff_controller],
            )
        ),
        odom_gt,
        robo_mapper,
        slammer,
        rviz_node,
        relay_cmd_vel
    ])