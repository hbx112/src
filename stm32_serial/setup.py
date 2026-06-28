from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'stm32_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 文件相关配置
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py')))
        # ('share/' + package_name, glob("launch/*.launch.py")),
        # ('share/' + package_name, glob("launch/xml/*.launch.xml")),
        # ('share/' + package_name, glob("launch/yaml/*.launch.yaml"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hbx',
    maintainer_email='hbx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stm32_ser = stm32_serial.stm32_ser:main'
        ],
    },
)
