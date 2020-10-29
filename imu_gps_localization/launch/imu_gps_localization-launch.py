import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer

import yaml

def generate_launch_description():

    project_dir = get_package_share_directory('imu_gps_localization')
    launch_dir = os.path.join(project_dir, 'launch')
    config_dir = os.path.join(project_dir, 'config')
    rviz_dir = os.path.join(project_dir, 'rviz')
    
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(rviz_dir, 'default.rviz'),
        description='Full path to the RVIZ config file to use')  
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
        
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')
        
        
    log_folder = {"log_folder": project_dir}
    param_config = os.path.join(config_dir, 'imu_gps_localization.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['imu_gps_localization_node']['ros__parameters']
    params.update(log_folder)
    imu_gps_localization_cmd = Node(
            package="imu_gps_localization",
            node_executable="imu_gps_localization_node",
            name="imu_gps_localization_node",
            output="screen",
            parameters=[
                params
            ]
            
        )
   
    
       
    # Create the launch description and populate
    ld = LaunchDescription()
    
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(imu_gps_localization_cmd)
    return ld
