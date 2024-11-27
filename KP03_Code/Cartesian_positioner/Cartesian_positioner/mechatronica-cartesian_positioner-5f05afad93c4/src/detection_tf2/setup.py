from setuptools import setup
import os
from glob import glob

package_name = 'detection_tf2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scwaanders',
    maintainer_email='437068@student.saxion.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tool_broadcaster = detection_tf2.static_weed_removal_tool_broadcaster:main',
            'static_camera_broadcaster = detection_tf2.static_camera_broadcaster:main',
            'transforming_coordinates = detection_tf2.transforming_coordinate_frames:main',
        ],
    },
)
