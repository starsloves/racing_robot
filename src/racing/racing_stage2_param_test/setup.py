from setuptools import setup
import os
from glob import glob

package_name = 'racing_stage2_param_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Standalone tester for stage2 inertial parameters',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'direct_inertial_tester = racing_stage2_param_test.direct_inertial_tester:main',
            'twist_cmd_relay = racing_stage2_param_test.twist_cmd_relay:main',
        ],
    },
)