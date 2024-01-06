import os

# launch文件中出现的argument和parameter，
# 虽都译为“参数”，但含义不同： - argument：仅限launch文件内部使用，方便在launch中调用某些数值； 
# - parameter：ROS系统的参数，方便在节点见使用某些数值。


# 获取功能包下share目录路径
from ament_index_python.packages import get_package_share_directory # 查询功能包路径的方法

from launch import LaunchDescription    # launch文件的描述类
from launch_ros.actions import Node     # 节点启动的描述类


# # 事件相关
from launch.event_handlers import OnProcessStart, OnProcessExit,OnExecutionComplete,OnProcessIO,OnShutdown
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.actions import TimerAction
from launch.conditions import UnlessCondition
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():      # 自动生成launch文件的函数
    # 找到配置文件的完整路径
    params_config = os.path.join(get_package_share_directory('distance_field'), 'config','nav2_params.yaml')

    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap_node',
        output='screen',
        parameters=[{'params_file': params_config, 'frame_id': 'map'},
                    {"autostart": True}],
        namespace='costmap',  # 添加命名空间
    )

    # 添加发布静态 TF 变换的节点
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        namespace='tf2_ros',
        remappings=[('/tf', '/tf_static')],
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  
        parameters=[
            {'autostart': True}, #设置自启动
            {'node_names': ['costmap_node']},  # 使用节点名称 "map_server"，而不是 "nav2_map_server"
            {'autowstart': True},  # 自动执行 start 操作
            {'autowconfigure': True},  # 自动执行 configure 操作
            {'initial_state': 'activate'},  # 设置初始状态为 "activate"
        ],
    )

    return LaunchDescription([lifecycle_manager, costmap_node, static_transform_publisher_node])