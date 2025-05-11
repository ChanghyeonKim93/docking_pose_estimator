import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  ld = LaunchDescription()
  
  # config = os.path.join(
  #   get_package_share_directory('ros2_tutorials'),
  #   'config',
  #   'params.yaml'
  # )
      
  stereo_vo_node = Node(
    package='docking_pose_estimator',
    executable='docking_pose_estimator_node',
    output='screen',
    parameters=[
        {'param1': '/stereo_camera/left/image_raw'},
        {'param2': '/directory/for/saving/pose'}
    ]
    # parameters = [config]
  )

  ld.add_action(stereo_vo_node)
  return ld