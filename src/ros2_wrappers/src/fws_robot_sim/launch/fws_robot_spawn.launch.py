import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. CONFIGURAZIONE PARAMETRI ---
    # Definiamo use_sim_time come LaunchConfiguration per poterlo passare ai nodi
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    fws_robot_description_path = os.path.join(
        get_package_share_directory('fws_robot_description'))
    
    fws_robot_sim_path = os.path.join(
        get_package_share_directory('fws_robot_sim'))

    # Impostiamo il path delle risorse per Gazebo
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(fws_robot_sim_path, 'worlds'), ':' +
            str(Path(fws_robot_description_path).parent.resolve())
            ]
        )

    # Argomenti di lancio
    world_arg = DeclareLaunchArgument(
        'world', 
        default_value='fws_robot_world',
        description='Gz sim World'
    )

    # --- 2. AVVIO DI GAZEBO ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 4', ' -r'])
        ]
    )

    # --- 3. ELABORAZIONE XACRO/URDF ---
    xacro_file = os.path.join(fws_robot_description_path,
                              'robots',
                              'fws_robot.urdf.xacro')

    # Elabora il file xacro
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')

    # --- 4. ROBOT STATE PUBLISHER ---
    # Importante: passiamo use_sim_time qui
    params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # --- 5. SPAWN DEL ROBOT ---
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0', '-y', '0.0', '-z', '0.07',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                   '-name', 'fws_robot',
                   '-allow_renaming', 'false'],
    )

    # --- 6. CONTROLLERS (FIX PRINCIPALE) ---
    # Usiamo il nodo 'spawner' invece di ExecuteProcess.
    # NON passare parametri aggiuntivi qui, solo il nome del controller.
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller"],
        output="screen",
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller"],
        output="screen",
    )

    # --- 7. BRIDGE (FIX CLOCK) ---
    # Aggiunto il mapping per il clock per sincronizzare ROS 2 e Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'  # <--- FONDAMENTALE
        ],
        output='screen'
    )

    # --- 8. RVIZ ---
    rviz_config_file = os.path.join(fws_robot_description_path, 'config', 'fws_robot_config.rviz')

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}] # Anche Rviz deve usare il tempo simulato
    )

    # --- 9. DEFINIZIONE DEL LAUNCH ---
    return LaunchDescription([
        gazebo_resource_path,
        world_arg,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        
        # Gestione sequenza di avvio:
        # 1. Quando finisce lo spawn -> avvia Joint State Broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        # 2. Quando parte Joint State (significa che il controller manager è vivo) -> avvia gli altri
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[forward_velocity_controller_spawner,
                         forward_position_controller_spawner],
            )
        ),
        rviz,
    ])