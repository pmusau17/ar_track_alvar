from rclpy.duration import Duration

import os
import sys
import time
import unittest
import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch

import pytest
import rclpy

pr2_bundle_no_kinect = os.path.join(get_package_share_directory('ar_track_alvar'),'launch','pr2_bundle_no_kinect.py')
pr2_bundle = os.path.join(get_package_share_directory('ar_track_alvar'),'launch','pr2_bundle.py')
pr2_indiv_no_kinect = os.path.join(get_package_share_directory('ar_track_alvar'),'launch','pr2_indiv_no_kinect.py')
pr2_indiv = os.path.join(get_package_share_directory('ar_track_alvar'),'launch','pr2_indiv.py')
pr2_train = os.path.join(get_package_share_directory('ar_track_alvar'),'launch','pr2_train.py')

bag_name = os.path.join(os.path.dirname(__file__),'resources','alvar-marker-pose-test.bag')

# pr2_bundle_no_kinect = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource(pr2_bundle_no_kinect))
# pr2_bundle = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource(pr2_bundle))
# pr2_indiv_no_kinect = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource(pr2_indiv_no_kinect))
# pr2_indiv = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource(pr2_indiv))
# pr2_train = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource(pr2_train))

@pytest.mark.rostest
def generate_test_description():
    return LaunchDescription([
        
        # Launch Bag
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play','--loop', '-s',"rosbag_v2",bag_name],
            output='screen'
        ),

        # include the child launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ar_track_alvar'), '/launch/pr2_indiv_no_kinect.py']),
        ),
        launch_testing.actions.ReadyToTest(),
    ])
    # return launch.LaunchDescription
    # ([
    #     # pr2_bundle_no_kinect,
    #     # pr2_bundle,
    #     # pr2_indiv_no_kinect,
    #     # pr2_indiv,
    #     # pr2_train,
    #     launch_testing.actions.ReadyToTest(),
    # ])

class TestArTrack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_ar_track_alvar_launch_files')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_launch_files(self):
        # Wait up to 10 seconds for all the nodes to launch
        launch_success=True
        start_time = time.time()
        while (time.time() - start_time) < 10:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        assert launch_success
