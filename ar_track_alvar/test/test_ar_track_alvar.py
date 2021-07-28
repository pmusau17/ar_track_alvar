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


bag_name = os.path.join(get_package_share_directory('ar_track_alvar'),'ar_track_alvar_4markers_tork_2017-02-08-11-21-14.bag')

@pytest.mark.rostest
def generate_test_description():

    return LaunchDescription([
        # DisparityNode


        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-s',"rosbag_v2",bag_name],
            output='screen'
        ),

        # Node(
        #     package='ros2bag',
        #     executable='bag',
        #     name='bag_node',
        #     arguments=["play","-s","rosbag_v2",bag_name],
        #     output='screen'
        # ),
        launch_testing.actions.ReadyToTest(),
    ])

class TestArTrack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_ar_track_alvar_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()


    def test_message_received(self):
        # Expect the disparity node to publish on '/disparity' topic
        msgs_received = []
        self.node.create_subscription(
            Image,
            '/camera/image_raw',
            lambda msg: msgs_received.append(msg),
            1
        )

        # Wait up to 60 seconds to receive message
        start_time = time.time()
        while len(msgs_received) == 0 and (time.time() - start_time) < 60:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        assert len(msgs_received) > 0

