from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='online_mesh_mapper',
            namespace='online_mesh_mapper',
            executable='online_mesh_mapper',
            name='online_mesh_mapper',
            parameters=[{
                'in_topic': '/all_sensor_global_pointcloud2', #input topic of type pointcloud_2
                'frame_id': 'odom', #frame id of the input pointcloud 2
                'odometry_msg_topic': '/Spot/odometry', #topic that outputs nav_msgs_msg_odometry
                'scalar': 8,
                'render_distance_horizontal':2,
                'render_distance_vertical':1,
                'out_topic':'/Nav/mesh_map',
                'max_chunks':1<<13,#this HAS to be a power of 2
                'obj_filepath':'/home/martin/Desktop'
            }],
        )
    ])
