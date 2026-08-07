from setuptools import setup
import os
from glob import glob

package_name = 'racing_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'web'), glob(os.path.join('web', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Racing debugging tools',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_recorder = racing_tools.data_recorder:main',
            'camera_video_recorder = racing_tools.camera_video_recorder:main',
            'initial_scan_map_localizer = racing_tools.initial_scan_map_localizer:main',
            'start_corner_pose_diagnostic = racing_tools.start_corner_pose_diagnostic:main',
            'manual_trajectory_recorder = racing_tools.manual_trajectory_recorder:main',
            'telemetry_web_monitor = racing_tools.telemetry_web_monitor:main',
        ],
    },
)
