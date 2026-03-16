from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yumi_rws_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        # Scripts
        (os.path.join('share', package_name, 'scripts'),
         glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=[
        'setuptools',
        'requests',
    ],
    zip_safe=True,
    maintainer='Rui Martins',
    maintainer_email='up202108756@edu.fe.up.pt',
    description='ROS 2 interface for ABB YuMi control via Robot Web Services',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = yumi_rws_interface.joint_state_publisher:main',
            'rws_commander = yumi_rws_interface.rws_commander:main',
            'rws_trajectory_controller = yumi_rws_interface.rws_trajectory_controller:main',
            'gripper_service = yumi_rws_interface.gripper_service:main',
            'gripper_action_server = yumi_rws_interface.gripper_action_server:main',
            'egm_trajectory_controller = yumi_rws_interface.egm_trajectory_controller:main',
        ],
    },
)
