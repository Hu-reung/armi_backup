from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'armi_navigation'  # navigation 대신 고유 이름 추천

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # ← 추가
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hci',
    maintainer_email='hci@todo.todo',
    description='Waypoint follower client for Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_follower = armi_navigation.waypoint_follower:main',
            'nav_to_pose_client = armi_navigation.nav_to_pose_client:main',
            'camera_node = armi_navigation.camera_node:main',
        ],
    },
)
