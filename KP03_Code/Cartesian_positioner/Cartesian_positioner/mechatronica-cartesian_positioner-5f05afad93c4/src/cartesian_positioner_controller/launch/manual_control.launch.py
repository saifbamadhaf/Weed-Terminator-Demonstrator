import os
import yaml
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file

def load_yaml_servo(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    urdf_file = get_package_file('cartesian_positioner_description', 'urdf/cartesian_positioner_manual_control.urdf')
    
    #urdf_file = get_package_file('cartesian_positioner_description', 'urdf/cartesian_positioner_fake_controllers.urdf')
    srdf_file = get_package_file('cartesian_positioner_moveit_config', 'config/cartesian_positioner.srdf')
    kinematics_file = get_package_file('cartesian_positioner_moveit_config', 'config/kinematics.yaml')
    ompl_config_file = get_package_file('cartesian_positioner_moveit_config', 'config/ompl_planning.yaml')
    moveit_controllers_file = get_package_file('cartesian_positioner_moveit_config', 'config/controllers.yaml')
    ros_controllers_file = get_package_file('cartesian_positioner_moveit_config', 'config/ros_controllers_joystick.yaml')

    robot_description = load_file(urdf_file)
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)

    servo_config_file = get_package_file("cartesian_positioner_moveit_config", "config/moveit_servo_config_manual_control.yaml")
    servo_yaml = load_yaml(servo_config_file)
    servo_params = {"moveit_servo": servo_yaml}

    robot_description_servo = {"robot_description": robot_description}

    robot_description_semantic_servo = {
        "robot_description_semantic": robot_description_semantic
    }


    moveit_controllers = {
        'moveit_simple_controller_manager' : load_yaml(moveit_controllers_file),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.1
    }
    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }

    # MoveIt node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
                'ompl': ompl_config,
                'planning_pipelines': ['ompl'],
            },
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_config,
        ],
    )

    # Visualization (parameters needed for MoveIt display plugin)
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # Controller manager for realtime interactions
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters= [
            {'robot_description': robot_description},
            ros_controllers_file
        ],
        output="screen",
        #prefix="gdbserver localhost:3000"

    )
    # Startup up ROS2 controllers (will exit immediately)
#    controller_names = ['manipulator_joint_trajectory_controller', 'joint_state_broadcaster', 'x_joint_comp_position_controller','joystick_position_controller','joystick_velocity_controller']
    #controller_names = ['manipulator_joint_trajectory_controller', 'joint_state_broadcaster', 'joystick_velocity_controller']
    controller_names = ['joint_state_broadcaster', 'joystick_position_controller']
    spawn_controllers = [
        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[controller, "-c", "/controller_manager"],
            output="screen")
        for controller in controller_names
    ]

    container = ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # TF information
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[
                    {'robot_description': robot_description}],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/root_link", "frame_id": "/world"}],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoServer",
                name="servo_server",
                parameters=[
                    servo_params,
                    robot_description_servo,
                    robot_description_semantic_servo,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="cartesian_positioner_controller",
                plugin="cartesian_positioner_controller::JoyToServoPub",
                name="controller_to_servo_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        #prefix='gdbserver localhost:3000'
    )

    positioner_controller = Node(
        name="positioner_controller",
        package="cartesian_positioner_controller",
        executable="positioner_controller",
        output="screen",
        parameters=[{'robot_description': robot_description}, {'robot_description_semantic': robot_description_semantic}, {'robot_description_kinematics': kinematics_config}, {"manual":True}],
        #prefix='gdbserver localhost:3000'
    )

    return LaunchDescription([
        ros2_control_node,
        move_group_node,
        container,
        positioner_controller
        #rviz,
        ] + spawn_controllers
    )