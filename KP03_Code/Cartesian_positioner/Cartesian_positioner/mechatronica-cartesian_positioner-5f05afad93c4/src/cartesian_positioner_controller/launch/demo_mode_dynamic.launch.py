import os
import yaml
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

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

def run_xacro(xacro_file, args):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} {args} -o {urdf_file}')
    return urdf_file

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    
    xacro_args, setup_type = process_arguments(sys.argv)

    declared_arguments = create_launch_args()

    use_rviz = LaunchConfiguration('rviz')
    only_sim = LaunchConfiguration('sim')

    robot_description_config = get_robot_description(xacro_args, setup_type)
    robot_description = {"robot_description": robot_description_config}

    srdf_file = get_package_file(
        "cartesian_positioner_moveit_config", "config/cartesian_positioner.srdf"
    )

    robot_description_semantic_config = load_file(srdf_file)

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "cartesian_positioner_moveit_config", "config/kinematics.yaml"
    )

    # MoveGroupInterface executable
    positioner_controller = Node(
        name="positioner_controller",
        package="cartesian_positioner_controller",
        executable="positioner_controller",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
       # prefix='gdbserver localhost:3000'
    )
     # Visualization (parameters needed for MoveIt display plugin)
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
        condition=IfCondition(use_rviz)
    )

    publish_world_tf = Node(
            name='frame_publisher',
            package='positioner_tf2_frame_publisher',
            executable='positioner_tf2_frame_publisher',
            output='screen',
            parameters=[
                {'topicname': 'conveyor'}
            ]
    )

    # MoveGroupInterface executable
    tracking_node = Node(
        name="plant_tracking",
        package="cartesian_positioner_controller",
        executable="plant_tracking",
        output="screen",
        #prefix='gdbserver localhost:3000'
    )
    detector = Node(
            package='detection_protocol',
            executable='detections',
            name='publisher_detections',
        )

    conveyor_reading = Node(
            package='positioner_tf2_frame_publisher',
            executable='conveyor_encoder',
            name='conveyor',
            parameters=[
                {'belt_speed': 0.025},
                {'belt_length': 3.545},
                {'publish_period_ms': 20}],
            condition=IfCondition(only_sim)
        )
    
    conveyor_sim = Node(
            package='conveyor_interface',
            executable='read_conveyor',
            name='conveyor',
            condition=UnlessCondition(only_sim)
        )
    return LaunchDescription(declared_arguments + [publish_world_tf, positioner_controller, tracking_node, detector, conveyor_reading, conveyor_sim, rviz])
