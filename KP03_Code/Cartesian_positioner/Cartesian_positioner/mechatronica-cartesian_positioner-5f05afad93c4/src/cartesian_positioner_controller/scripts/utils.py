import yaml
import os
import odrive

from ament_index_python import get_package_share_directory

def open_odrive(odrive_serials, setup_type):
    odrvXA = []
    odrvYZ = []
    connected = False

    #try to find odrives
    try:
        odrvXA = odrive.find_any(serial_number=odrive_serials['configurations'][setup_type]['serial_xa'], timeout=3)
        odrvYZ = odrive.find_any(serial_number=odrive_serials['configurations'][setup_type]['serial_yz'], timeout=3)
        connected = True
    except TimeoutError:
        print("No", setup_type, "ODrives found")

    if (connected):
        print(setup_type, "ODrives found!")
        odrive_serials['last'] = setup_type
        save_odrive_serials(odrive_serials)
    return connected, odrvXA, odrvYZ

def connect_odrives():
    odrive_serials = get_odrive_serials()

    connected = False

    print("Trying to connect to", odrive_serials['last'], "(last used)")
    connected, odrvXA, odrvYZ = open_odrive(odrive_serials, odrive_serials['last'])
    if (connected):
        return odrvXA, odrvYZ    

    print("Searching for connected ODrives...")

    for setup_type in odrive_serials['configurations']:
        connected, odrvXA, odrvYZ = open_odrive(odrive_serials, setup_type)
        if (connected):
            return odrvXA, odrvYZ   
    
    if (not connected):
        raise TimeoutError("No ODrives found, check connections!")
    return [],[]


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

def save_yaml(data, file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'w') as file:
            yaml.safe_dump(data, file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        print("Error saving yaml")

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

def save_odrive_serials(data):
    #retrieve serial numbers for odrives    
    odrive_serials_file = get_package_file('cartesian_positioner_description', 'config/odrive_serials.yaml')
    save_yaml(data, odrive_serials_file)

def get_robot_description(xacro_args, setup_type):
    #get serial numbers and run xacro
    odrive_serials = get_odrive_serials()
    xacro_args += " odrive_serial_xa:=%s odrive_serial_yz:=%s" % (odrive_serials[setup_type]['serial_xa'], odrive_serials[setup_type]['serial_yz'])

    xacro_file = get_package_file('cartesian_positioner_description', 'urdf/cartesian_positioner.urdf.xacro')

    urdf_file = run_xacro(xacro_file, xacro_args)
    return load_file(urdf_file)

def process_arguments(arg_list):
    xacro_args = ''
    setup_type = ''
    for arg in arg_list:
        if arg.startswith("sim:="):
            xacro_args += " "+arg
        if arg.startswith("a_axis_offset:="):
            xacro_args = " "+arg
        if arg.startswith("setup_type:="):
            setup_type = arg[12:]
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
            "setup_type",
            description="Choose setup type to run software for (either 'conveyor-weeder' or 'mobile-weeder')"
        )
    )

    return declared_arguments