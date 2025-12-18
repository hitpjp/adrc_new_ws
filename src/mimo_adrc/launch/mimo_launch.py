import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 1. 定义 ADRC 仿真节点
    mimo_node = Node(
        package='mimo_adrc',
        executable='mimo_node',
        name='adrc_sim_node',
        output='screen'
    )

    # 2. 定义 PlotJuggler 绘图工具启动指令
    # 我们使用 ExecuteProcess 来运行 'ros2 run plotjuggler plotjuggler'
    plotjuggler = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
        output='screen'
    )

    # 3. 同时返回这两个动作
    return LaunchDescription([
        mimo_node,
        plotjuggler
    ])