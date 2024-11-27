import os
import yaml
import sys
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory

#add current directory to be able to import launch_utils file
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from launch_utils import get_robot_description, process_arguments

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

def run_xacro(xacro_file, args):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} {args} -o {urdf_file}')
    return urdf_file


def generate_launch_description():
      
    xacro_args, setup_type = process_arguments(sys.argv)

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="false"
        )
    )
    
    srdf_file = get_package_file('cartesian_positioner_moveit_config', 'config/cartesian_positioner.srdf')
    kinematics_file = get_package_file('cartesian_positioner_moveit_config', 'config/kinematics.yaml')
    
    robot_description = get_robot_description(xacro_args, setup_type)
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)



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


    return LaunchDescription(declared_arguments + 
        [rviz,]
    )