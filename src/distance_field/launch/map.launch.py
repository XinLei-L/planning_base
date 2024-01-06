import os

# launch文件中出现的argument和parameter，
# 虽都译为“参数”，但含义不同： - argument：仅限launch文件内部使用，方便在launch中调用某些数值； 
# - parameter：ROS系统的参数，方便在节点见使用某些数值。


# 获取功能包下share目录路径
from ament_index_python.packages import get_package_share_directory # 查询功能包路径的方法

from launch import LaunchDescription    # launch文件的描述类
from launch_ros.actions import Node     # 节点启动的描述类

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():      # 自动生成launch文件的函数
    # 找到配置文件的完整路径
    rviz_config = os.path.join(get_package_share_directory('distance_field'), 'config', 'distance_field.rviz')
    map_config = os.path.join(get_package_share_directory('distance_field'), 'maps','b.yaml')

    rviz2 = Node (
        package="rviz2", 
        executable="rviz2",
        name="rviz2", 
        arguments=['-d', rviz_config]
    ) 

    nav2_map_server = Node (
        package="nav2_map_server",
        executable="map_server",
        parameters=[
          {"frame_id": "map", "yaml_filename" : map_config},
          {"autostart": True}, #设置自启动
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  
        parameters=[
            {'autostart': True}, #设置自启动
            {'node_names': ['map_server']},  # 使用节点名称 "map_server"，而不是 "nav2_map_server"
            {'initial_state': 'activate'},  # 设置初始状态为 "activate"
        ],
       
    )

    # distance_field_node = Node(
    #     package="distance_field",
    #     executable="distance_field_node"
    # )

    return LaunchDescription([rviz2, nav2_map_server,lifecycle_manager])