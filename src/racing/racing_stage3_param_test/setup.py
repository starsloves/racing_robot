from setuptools import setup
import os
from glob import glob

package_name = 'racing_stage3_param_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Stage 3 param test — map-frame return navigation to P point',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stage3_return_navigator = racing_stage3_param_test.stage3_return_navigator:main',
            'phase3_test_trigger = racing_stage3_param_test.phase3_test_trigger:main',
            'simple_return_navigator = racing_stage3_param_test.simple_return_navigator:main',
        ],
    },
)
