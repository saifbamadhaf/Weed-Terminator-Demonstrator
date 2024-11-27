import yaml
import os
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument

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


def get_odrive_serials():
    #retrieve serial numbers for odrives    
    odrive_serials_file = get_package_file('cartesian_positioner_description', 'config/odrive_serials.yaml')
    return load_yaml(odrive_serials_file)

def get_robot_description(xacro_args, setup_type):
    #get serial numbers and run xacro
    odrive_serials = get_odrive_serials()
    if (setup_type != "sim"):
        xacro_args += " odrive_serial_xa:=%s odrive_serial_yz:=%s" % (odrive_serials['configurations'][setup_type]['serial_xa'], odrive_serials['configurations'][setup_type]['serial_yz'])

    xacro_file = get_package_file('cartesian_positioner_description', 'urdf/cartesian_positioner.urdf.xacro')

    urdf_file = run_xacro(xacro_file, xacro_args)
    return load_file(urdf_file)

def process_arguments(arg_list):
    xacro_args = ''
    setup_type = ''
    for arg in arg_list:
        if arg.startswith("sim:="):
            setup_type = 'sim'
        if arg.startswith("a_axis_offset:="):
            xacro_args = " "+arg
        if arg.startswith("setup_type:=") and setup_type != "sim":
            setup_type = arg[12:]
    
    if (setup_type=="sim"):
        xacro_args += " sim:=true"
        
    if (setup_type == ''):
        serials = get_odrive_serials()
        types = list(serials['configurations'].keys())
        types.append("sim")
        raise RuntimeError(f'setup_type is not provided. Choose from: {types}')
    return xacro_args, setup_type

def create_launch_args():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="false"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "manual",
            default_value="false"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "setup_type",
            description="Choose setup type to run software for (either 'conveyor-weeder' or 'mobile-weeder')"
        )
    )

    return declared_arguments