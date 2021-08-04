

from launch import LaunchDescription
import launch_ros.actions
import os 

# command line arguments 
marker_size=4.4
max_new_marker_error=0.08
max_track_error=0.2
cam_image_topic="wide_stereo/left/image_color"
cam_info_topic="wide_stereo/left/camera_info"	
output_frame="torso_lift_link"

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ar_track_alvar', 
            executable='individualMarkersNoKinect', 
            output='screen',
            remappings=[
                ("camera_image", cam_image_topic),
                ("camera_info",cam_info_topic)
            ],
            parameters=[
                {"marker_size":marker_size},
		        {"max_new_marker_error":max_new_marker_error},
		        {"max_track_error":max_track_error},
		        {"output_frame":output_frame}
            ],)
    ])