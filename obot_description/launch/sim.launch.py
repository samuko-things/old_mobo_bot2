import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, ExecuteProcess
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown

import xacro








# ign service -s /world/plane_world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "obot.urdf", name: "diff_bot"'




def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

     # declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )

    # Path Variables 
    pkg_path = os.path.join(get_package_share_directory('obot_description'))

    ignition_ros_package = os.path.join(get_package_share_directory("ros_gz_sim"))
    simulation_world_file = os.path.join(pkg_path, "world/empty.sdf")
    rviz_config_file = os.path.join(pkg_path,'config','robot_view.rviz')
    
    # xacro_file = os.path.join(pkg_path,'urdf','robot_urdf.xacro')
    # robot_description_config= Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control])
    # robot_description_config= Command(['xacro ', xacro_file])
    # robot_description_xml = ParameterValue(robot_description_config, value_type=str)
    # params = {'robot_description': robot_description_xml, 'use_sim_time': use_sim_time}

    urdf_file = os.path.join(pkg_path, 'urdf', '_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_xml = infp.read()
    params = {'robot_description': robot_description_xml, 'use_sim_time': use_sim_time}
  
    # Create a robot_state_publisher node
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )


    simulation = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', simulation_world_file],
        output='screen'
    )

    ### Can't use with event handlers
    # simulation = IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource(
    #                     launch_file_path=os.join.path(ignition_ros_package, "launch/gz_sim.launch.py").as_posix()
    #                 ),
    #                 launch_arguments={"gz_args": "-r " + simulation_world_file}.items()
    #             )

    # urdf_file = "obot.urdf"
    add_urdf_file = f'sdf_filename: "{urdf_file}", name: "obot"'
    spawn_robot_in_sim_world = ExecuteProcess(
        cmd=['ign', 'service', '-s', '/world/empty/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1000', '--req', add_urdf_file],
        output='screen'
    )
        
    ign_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/model/obot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                   "/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
                   "/model/obot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"], 
        # ros_arguments=["-r /model/wheeled_model/cmd_vel:=/cmd_vel"], # Remapping topic in terminal 
        remappings=[("/model/obot/cmd_vel", "/cmd_vel"),
                    ("/lidar", "/laser_scan"),
                    ("/model/obot/odometry", "/odom")], # Remapping topic in launch file
        output="screen"
    )

    spawn_robot_after_sim_has_started = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=simulation,
            on_start=[spawn_robot_in_sim_world],
        )
    )
    
    simulation_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=simulation,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )



     # Create the launch description and populate
    ld = LaunchDescription()

    # add the necessary declared launch arguments to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add the nodes to the launch description
    ld.add_action(rviz_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(simulation)
    ld.add_action(spawn_robot_after_sim_has_started)
    ld.add_action(ign_bridge_node)
    ld.add_action(simulation_exit)

    
    return ld      # return (i.e send) the launch description for excecution