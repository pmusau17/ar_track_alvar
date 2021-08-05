

from launch import LaunchDescription
import launch_ros.actions
import os 

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ar_track_alvar', 
            executable='individualMarkers', 
            output='screen',
            remappings=[
                ("camera_image", "/kinect_head/depth_registered_points"),
                ("camera_info","/kinect_head/rgb/camera_info")
            ],
            parameters=[
                {"marker_size":4.4},
		        {"max_new_marker_error": 0.08},
		        {"max_track_error":0.2},
		        {"output_frame":"torso_lift_link"}
            ],
        )
    ])