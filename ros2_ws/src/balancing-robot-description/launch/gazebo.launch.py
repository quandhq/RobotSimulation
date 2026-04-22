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
    
    #--- 2. Start the robot state publiser ---
    # The "Nervous system": Publish URDF's joint positions to the whole ROS network
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output= 'screen', 
        parameters=[{'robot_description': robot_desc}]
    ) 

    #--- 3. Start Gazebo ---
    # The "Physics" gym: Loads the standard Gazebo simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    #--- 4. Spawn the Robot ---
    # The "Hand": take URDF and drop it into Gazebo
    # Note the '-z', '0.1' argument. We spawn it 10cm in the air so it doesn't clip the floor!
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'balancing_robto', '-z', '0.1'],
        output='screen'        
    )
    #--- 5. Launch everything ---
    return LaunchDescription([
        rsp_node,
        gazebo,
        spawn_entity
    ])