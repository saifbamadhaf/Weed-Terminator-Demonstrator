from setuptools import setup
import os
from glob import glob
package_name = 'detection_protocol'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('detection_protocol/config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scwaanders',
    maintainer_email='scwaanders@gmail.com or 437068@student.saxion.nl',
    description='This package contains a topic that can be used to publish positions of detections.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detections = detection_protocol.publisher_detections:main',
            'pixelfarming_detections = detection_protocol.pixelfarming_translator:main',
            'static_publish_once = detection_protocol.publisher_static_once:main',
            'measurement_service = detection_protocol.measure_positions:main',
            'weeding_sequence = detection_protocol.weeding_sequence:main',
            'filtering = detection_protocol.filtering_detections:main',
            'speed = detection_protocol.publisher_speed:main',
            'time_stamp = detection_protocol.time_stamp:main'
        ],
    },
)
