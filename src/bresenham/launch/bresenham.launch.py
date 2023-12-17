import os

# launch文件中出现的argument和parameter，
# 虽都译为“参数”，但含义不同： - argument：仅限launch文件内部使用，方便在launch中调用某些数值； 
# - parameter：ROS系统的参数，方便在节点见使用某些数值。


# 获取功能包下share目录路径
from ament_index_python.packages import get_package_share_directory # 查询功能包路径的方法
# 参数声明与获取
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument       # 声明launch文件内使用的Argument类

from launch import LaunchDescription    # launch文件的描述类
from launch_ros.actions import Node     # 节点启动的描述类

# 封装终端指令相关类
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

# 事件相关
from launch.event_handlers import OnProcessStart, OnProcessExit,OnExecutionComplete,OnProcessIO,OnShutdown
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo



def generate_launch_description():      # 自动生成launch文件的函数
    # 找到配置文件的完整路径
    rviz_config = os.path.join(get_package_share_directory('bresenham'), 'config', 'bresenham.rviz')
    map_config = os.path.join(get_package_share_directory('bresenham'), 'map','b.yaml')

    rviz2 = Node (
        package="rviz2", 
        executable="rviz2",
        name="rviz2", 
        output="screen", 
        arguments=['-d', rviz_config]
    ) 

    nav2_map_server = Node (
        package="nav2_map_server",
        executable="map_server",
        parameters=[{"frame_id": "map", "yaml_filename" : map_config}]
    )

    # 配置环境 --configure --activate nav_map_server这个功能包里面的节点有启动的先后顺序！
    configure = ExecuteProcess(
        cmd=[FindExecutable(name="ros2"),' lifecycle ','set ','/map_server ','configure'],
        shell=True
    )
    activate = ExecuteProcess(
        cmd=[FindExecutable(name="ros2"),' lifecycle ','set ','/map_server ','activate'],
        shell=True
    )
    # 事件发生器 ---先打开rviz，然后进行config，然后进行activa
    configure_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rviz2,
            on_start=configure                     
        )
    )
    activate_event = RegisterEventHandler(
        event_handler=OnExecutionComplete(
            target_action=configure,
            on_completion=[
                LogInfo(msg='rviz map_server configure finish'),
                activate
            ]
        )
    )

    return LaunchDescription([nav2_map_server, configure_event, activate_event, rviz2])