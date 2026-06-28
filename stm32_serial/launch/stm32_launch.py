from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 获取两个功能包路径
    lslidar_driver_dir = get_package_share_directory("lslidar_driver")
    stm32_serial_dir = get_package_share_directory("stm32_serial")

    # 2. 包含激光雷达lsn10p自带launch文件
    launch_lsn10p = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lslidar_driver_dir, "launch", "lsn10p_launch.py")
        )
    )

    # 3. 启动STM32串口节点 stm32_ser.py
    node_stm32_serial = Node(
        package="stm32_serial",
        executable="stm32_ser",
        name="stm32_serial_node",
        output="screen",
    )

    # 整合所有启动项
    return LaunchDescription([
        launch_lsn10p,
        node_stm32_serial
    ])