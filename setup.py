from setuptools import setup
import os
import glob

package_name = 'robot_motion_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/motion_servers.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='sushanth.jayanth@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_motion_server_node = robot_motion_server.robot_motion_server:MotionServer',
            'nav2_client_node = robot_motion_server.robot_motion_server:nav2_client',
            'robot_pose_client_node = robot_motion_server.robot_motion_server:get_pose_client',
            'robot_docking_undocking_server_node = robot_motion_server.robot_motion_server:DockUndockServer',
            'custom_navigator_node = robot_motion_server.custom_navigator:CustomNavigator'
        ],
    },
)
