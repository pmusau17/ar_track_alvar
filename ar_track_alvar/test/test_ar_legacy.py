from rclpy.duration import Duration

import os
import sys
import time
import unittest

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
bag_name = os.path.join(get_package_share_directory('ar_track_alvar'),'ar_track_alvar_4markers_tork_2017-02-08-11-21-14.bag')
cam_image_topic ="camera/image_raw" 
cam_info_topic = "camera/camera_info" 	
marker_margin="2"
marker_resolution="5"
marker_size="2.3"
max_new_marker_error="0.08"
max_frequency="100"
max_track_error="0.2"
output_frame="camera"
        
@pytest.mark.rostest
def generate_test_description():

    return LaunchDescription([
        # Bag Node

        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-s',"rosbag_v2",bag_name],
            output='screen'
        ),

        Node(
            package='ar_track_alvar',
            executable='individualMarkersNoKinect',
            name='individual_markers',
            remappings=[
                ("camera_image", cam_image_topic),
                ("camera_info",cam_info_topic)
            ],
            arguments=[marker_size, max_new_marker_error, max_track_error, cam_image_topic, cam_info_topic, output_frame],
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


    def test_markers_received(self):
        # # Expect the disparity node to publish on '/disparity' topic
        # msgs_received = []
        # self.node.create_subscription(
        #     AlvarMarkers,
        #     "ar_pose_marker",
        #     lambda msg: msgs_received.append(msg),
        #     1
        # )
        start_time = time.time()
        while  rclpy.ok() and (time.time() - start_time) < 120:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            now = self.node.get_clock().now()

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
                    (trans, rot) = self.tfBuffer.lookup_transform('camera', target_frame, now, dur)
                    break
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                        self.node.get_logger().error(str(e) + ' target_frame={}'.format(target_frame))
                        continue

                # Compare each translation element (x, y, z) 
                for v_ret, v_expected in zip(trans, tf_expected[i][0]):
                    # Given that sigfig ignores the leading zero, we only compare the first sigfig.
                    numpy.testing.assert_approx_equal(
                        v_ret, v_expected, significant=1)
                # Compare each orientation element (x, y, z, w) 
                for v_ret, v_expected in zip(rot, tf_expected[i][1]):
                    numpy.testing.assert_approx_equal(
                        v_ret, v_expected, significant=1)

