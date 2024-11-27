from setuptools import setup

package_name = 'detection_protocol'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scwaanders',
    maintainer_email='scwaanders@gmail.com or 47068@student.saxion.nl',
    description='This package contains a topic that can be used to publish positions of detections.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = detection_protocol.publisher_member_function:main',
            'listener = detection_protocol.subscriber_member_function:main',
        ],
    },
)
