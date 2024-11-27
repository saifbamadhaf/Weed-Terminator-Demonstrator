import os
import yaml
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

#add current directory to be able to import launch_utils file
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from launch_utils import get_robot_description, process_arguments, create_launch_args

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

def load_yaml_servo(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    
    declared_arguments = create_launch_args()

    xacro_args, setup_type = process_arguments(sys.argv)

    robot_description = get_robot_description(xacro_args, setup_type)

    srdf_file = get_package_file('cartesian_positioner_moveit_config', 'config/cartesian_positioner.srdf')

    manual_control = LaunchConfiguration('manual')
    kinematics_file = get_package_file('cartesian_positioner_moveit_config', 'config/kinematics.yaml')
    ompl_config_file = get_package_file('cartesian_positioner_moveit_config', 'config/ompl_planning.yaml')
    pilz_config_file = get_package_file('cartesian_positioner_moveit_config', 'config/pilz_industrial_motion_planner_planning.yaml')
    pilz_limits_file = get_package_file('cartesian_positioner_moveit_config', 'config/pilz_cartesian_limits.yaml')
    moveit_controllers_file = get_package_file('cartesian_positioner_moveit_config', 'config/controllers.yaml')
    ros_controllers_file = get_package_file('cartesian_positioner_moveit_config', 'config/ros_controllers.yaml')

    
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)
    pilz_config = load_yaml(pilz_config_file)
    pilz_cartesian_limits = load_yaml(pilz_limits_file)

    servo_config_file = get_package_file("cartesian_positioner_moveit_config", "config/moveit_servo_config_manual_control.yaml")
    servo_yaml = load_yaml(servo_config_file)
    servo_params = {"moveit_servo": servo_yaml}

    touch_config_file = get_package_file("cartesian_positioner_controller", "config/touch_parameters.yaml")
    touch_params = load_yaml(touch_config_file)

    # servo_yaml = load_yaml("cartesian_positioner_moveit_config", "config/moveit_servo_config.yaml")
    # servo_params = {"moveit_servo": servo_yaml}

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
                'robot_description_planning': pilz_cartesian_limits,
                'ompl': ompl_config,
                'pilz':pilz_config,
                'planning_plugin':'pilz_command_planner',
                'default_planning_pipeline':'ompl',
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
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
            }
        ],
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
    controller_names = ['joint_state_broadcaster', 'x_joint_comp_position_controller']
    spawn_controllers = [
        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[controller, "-c", "/controller_manager"],
            output="screen")
        for controller in controller_names
    ]
    
    manual_controller_stopped = Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=['joystick_position_controller', "-c", "/controller_manager", "--stopped"],
            output="screen",
            condition=UnlessCondition(manual_control))
   
    manual_controller_active = Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=['joystick_position_controller', "-c", "/controller_manager"],
            output="screen",
            condition=IfCondition(manual_control))
    
    measurement_node = Node(
            package="detection_protocol",
            executable="measurement_service",          
            output="screen",
            condition=IfCondition(manual_control))
    
    positioner_controller = Node(
        name="positioner_controller",
        package="cartesian_positioner_controller",
        executable="positioner_controller",
        output="screen",
        parameters=[
                    {
                        'robot_description': robot_description,
                        'robot_description_semantic': robot_description_semantic,
                        'manual': True,
                    },
                    kinematics_config,
                    touch_params
                    ],
        condition=IfCondition(manual_control))

    traj_controller_stopped = Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=['manipulator_joint_trajectory_controller', "-c", "/controller_manager", "--stopped"],
            output="screen",
            condition=IfCondition(manual_control))
   
    traj_controller_active = Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=['manipulator_joint_trajectory_controller', "-c", "/controller_manager"],
            output="screen",
            condition=UnlessCondition(manual_control))
    
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
                parameters=[
                    touch_params,
                ],
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
    )

    return LaunchDescription(declared_arguments + [
        move_group_node,
        ros2_control_node,
        container,
        traj_controller_active,
        traj_controller_stopped,
        manual_controller_active,
        manual_controller_stopped,
        measurement_node,
        positioner_controller
        # rviz,
        ] + spawn_controllers
    )