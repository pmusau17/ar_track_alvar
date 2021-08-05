from rclpy.duration import Duration

import os
import sys
import time
import unittest
import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch

import pytest
import rclpy

from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf2_ros import TransformBroadcaster, TransformListener, TransformStamped, Buffer, LookupException, ConnectivityException, ExtrapolationException

# Test Parameters
bag_name = os.path.join(os.path.dirname(__file__),'resources','alvar-marker-pose-test.bag')
@pytest.mark.rostest
def generate_test_description():

    return LaunchDescription([
        # Launch bag with four markers
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '--loop','-s',"rosbag_v2",bag_name],
            output='screen'
        ),
        
        # Invidividual Marker Detector
        Node(
            package='ar_track_alvar',
            executable='individualMarkersNoKinect',
            name='individual_markers',
            remappings=[
                ("camera_image", "camera/image_raw"),
                ("camera_info", "camera/camera_info")
            ],
            parameters=[
                {"marker_size":2.3},
		        {"max_new_marker_error":0.08},
		        {"max_track_error":0.2},
		        {"output_frame":"camera"},
                {"max_frequency":100.0},
                {"marker_margin":2},
                {"marker_resolution":5}
            ],
            output='screen'
        ),
        launch_testing.actions.ReadyToTest(),
    ])

class TestArTrack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.tfBuffer = Buffer()
        cls.node = rclpy.create_node('test_ar_track_alvar_markers')
        cls.tflistener = TransformListener(cls.tfBuffer, cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()


    def test_marker_pose_estimation(self):
        transforms={}
        start_time = time.time()
        count = 0
        while  rclpy.ok() and (time.time() - start_time) < 120:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            dur = Duration()
            dur.sec = 40
            dur.nsec = 0
            
            #The values in this list are ordered in the marker's number. 
            tf_expected = [[[0.04464459977845401, 0.05341412598745035, 0.26796900627543024], [0.6726999185589797, 0.7391474391806558, -0.01739267319701629, -0.028868280906256056]],
                        [[-0.04805772245624329, 0.039528315926071665, 0.26775882622136327], [0.48207151562664247, 0.8758763282975102, -0.016363763970395625, -0.013414118615296202]],
                        [[0.007233278235745441, 0.015615692018491452, 0.26619586686955365], [0.08546919545682985, 0.9959809257461487, 0.00424040439, -0.02677659572186436]],
                        [[0.06223014382428272, 0.014613815037010106, 0.26226145707174475], [-0.46400320825216246, 0.8850390875261293, 0.032644264656690035, -0.018471282241381157]]]
            
            for i in range (0, len(tf_expected)):
                try:
                    target_frame = 'ar_marker_{}'.format(i)
                    time_stamp = rclpy.time.Time(seconds=0, nanoseconds=0).to_msg()
                    obj  = self.tfBuffer.lookup_transform('camera', target_frame,time=time_stamp)
                    trans = obj.transform.translation
                    rot = obj.transform.rotation
                    
                    item = [[trans.x,trans.y,trans.z],[rot.x,rot.y,rot.z,rot.w]]
                    transforms[i] = item
                    
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                        continue

            # Break when we've received all four markers
            if(len(transforms)==4):
                 break
                
        # There are four markers in the test bag. 
        # Make sure the deviation in rotation and translation is less than 0.15 for translation, 0.1 for orientation
        if(len(transforms)>=4):
            for i in range(4):
                trans = np.abs(np.asarray(transforms[i][0]) - np.asarray(tf_expected[i][0]))
                rot = np.abs( np.asarray(transforms[i][1]) - np.asarray(tf_expected[i][1]))
                assert np.max(trans) < 0.16
                assert np.max(rot) < 0.1




