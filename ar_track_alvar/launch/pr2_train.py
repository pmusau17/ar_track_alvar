

from launch import LaunchDescription
from launch_ros.actions import Node
import os 

# command line arguments 
nof_markers="8"
marker_size="4.4"
max_new_marker_error="0.08"
max_track_error="0.2"
cam_image_topic="/wide_stereo/left/image_color"
cam_info_topic="/wide_stereo/left/camera_info"	
output_frame="/torso_lift_link"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ar_track_alvar', 
            executable='trainMarkerBundle', 
            output='screen',
            remappings=[
                ("camera_image", cam_image_topic),
                ("camera_info",cam_info_topic)
            ],
            arguments=[nof_markers, marker_size, max_new_marker_error, max_track_error, cam_image_topic, cam_info_topic, output_frame]
        ),
    ])