from setuptools import setup
import os
from glob import glob

package_name = 'qr_scanner'

setup(
    name=package_name,
    version='0.0.1',
    # 这里的 packages 确保包含你的源码文件夹
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- 新增：包含 launch 文件夹下的所有启动文件 ---
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
        # --- 保持：包含 config 文件夹下的所有配置文件 ---
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='QR code scanner node for racing car',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 确保这里的路径指向你的脚本中的 main 函数
            'qr_scanner = qr_scanner.qr_scanner:main',
        ],
    },
)