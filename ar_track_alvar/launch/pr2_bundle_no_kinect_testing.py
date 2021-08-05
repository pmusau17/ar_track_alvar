from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 

# command line arguments 
marker_size="4.4"
max_new_marker_error="0.08"
max_track_error="0.2"
cam_image_topic="/wide_stereo/left/image_color"
cam_info_topic="/wide_stereo/left/camera_info"	
output_frame="/torso_lift_link"
truth_table_leg = os.path.join(get_package_share_directory('ar_track_alvar'),'bundles','truthTableLeg.xml')
table_bundle = os.path.join(get_package_share_directory('ar_track_alvar'),'bundles','table_8_9_10.xml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ar_track_alvar', 
            name='findMarkerBundlesNoKinect',
            executable='findMarkerBundlesNoKinect', 
            output='screen',
            remappings=[
                ("camera_image", cam_image_topic),
                ("camera_info",cam_info_topic)
            ],
            arguments=[marker_size, max_new_marker_error, max_track_error, cam_image_topic, cam_info_topic, output_frame, truth_table_leg, table_bundle,"true"],
        ),
    ])