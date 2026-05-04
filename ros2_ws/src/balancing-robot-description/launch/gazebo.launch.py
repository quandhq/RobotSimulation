import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    #--- 1. Locate your package and URDF ---
    # NOTE: Ensure this matches the <name> tag on package.xml
    pkg_name = 'balancing-robot-description'
    pkg_share = get_package_share_directory(package_name=pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    # Read the URDF file so we can pass it to the nodes
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        print(robot_desc)

    #Replace the '$(find balancing-robot-description)' with the absolute path to 'controllers.yaml' file 
    robot_desc = robot_desc.replace('$(find balancing-robot-description)', pkg_share)
    
    #--- 2. Start the robot state publiser ---
    # The "Nervous system": Publish URDF's joint positions to the whole ROS network
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output= 'screen', 
        parameters=[{'robot_description': robot_desc}]
    ) 

    # 3. Start Gazebo Sim with a ground plane
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        # 'empty.sdf' sometimes lacks a collision floor in certain versions
        # 'default' or explicitly adding a ground plane is safer
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    #--- 4. Spawn the Robot ---
    # The "Hand": take URDF and drop it into Gazebo
    # Note the '-z', '0.1' argument. We spawn it 10cm in the air so it doesn't clip the floor!
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'balancing_robot', '-z', '0.2'],
        output='screen'        
    )
    # ---5. The Bridge (The Interpreter)---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist', #no longer transferring velocity to control motors
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/balancing_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    # --- 6. controller_manager spawners ---
    # This activates the joint_state_broadcaster (Sensors)
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # This activates the effort_controller (Actuators)
    load_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--controller-manager", "/controller_manager"],
    )

    #---7. controller node---
    balancing_controller = Node(
        package='balancing_controller',
        executable='balancing_controller',
        output='screen',
        parameters=[{'use_sim_time':True}],
    )
    #--- 7. Launch everything ---
    return LaunchDescription([
        rsp_node,
        gazebo,
        spawn_entity,
        bridge,
        balancing_controller,
        load_joint_state_broadcaster,
        load_effort_controller
    ])