/*
 Software License Agreement (BSD License)

 Copyright (c) 2012, Scott Niekum
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the Willow Garage nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 author: Scott Niekum
*/


#include <std_msgs/msg/bool.hpp>
#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/msg/alvar_marker.hpp>
#include <ar_track_alvar_msgs/msg/alvar_markers.hpp>
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>

using namespace alvar;
using namespace std;

class IndividualMarkersNoKinect : public rclcpp::Node
{
  

  private:
    bool init=true;
    Camera *cam;
    cv_bridge::CvImagePtr cv_ptr_;

    ar_track_alvar_msgs::msg::AlvarMarkers arPoseMarkers_;
    visualization_msgs::msg::Marker rvizMarker_;
    

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr  cam_sub_;
    

    // image_transport::Subscriber cam_sub_;

 
    rclcpp::Publisher<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr arMarkerPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rvizMarkerPub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

    tf2::TimePoint prev_stamp_;
    std::shared_ptr<tf2_ros::Buffer> tf2_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    MarkerDetector<MarkerData> marker_detector;

    double parameters_set = false;
    rclcpp::TimerBase::SharedPtr timer_;
    double max_frequency=8.0;
    double marker_size;
    double max_new_marker_error;
    double max_track_error;
    std::string cam_image_topic;
    std::string cam_info_topic;
    std::string output_frame;
    int marker_resolution = 5; // default marker resolution
    int marker_margin = 2; // default marker margin

    
  public:
    IndividualMarkersNoKinect(int argc, char* argv[]) : Node("marker_detect") //, tf2_(this->get_clock()), tf_listener_(tf2_), tf_broadcaster_(this)//, it_(this)
    {

        rclcpp::Clock::SharedPtr clock = this->get_clock();
        tf2_ = std::make_shared<tf2_ros::Buffer>(clock);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
 
        // Get parameters.
        this->declare_parameter<double>("marker_size", 10.0);
        this->declare_parameter<double>("max_new_marker_error", 0.08);
        this->declare_parameter<double>("max_track_error", 0.2);
        this->declare_parameter<double>("max_frequency", 8.0);
        this->declare_parameter<int>("marker_resolution", 5);
        this->declare_parameter<int>("marker_margin", 2);
        this->declare_parameter<std::string>("output_frame", "");


        // Camera input topics. Use remapping to map to your camera topics.
        cam_image_topic = "camera_image";
        cam_info_topic = "camera_info";

        marker_detector.SetMarkerSize(marker_size, marker_resolution, marker_margin);
	      cam = new Camera();
      
        prev_stamp_ = tf2::get_now();

        //Give tf a chance to catch up before the camera callback starts asking for transforms
        // It will also reconfigure parameters for the first time, setting the default values
        //TODO: come back to this, there's probably a better way to do this 
        rclcpp::Rate loop_rate(100);
        loop_rate.sleep();

        arMarkerPub_ = this->create_publisher<ar_track_alvar_msgs::msg::AlvarMarkers> ("ar_pose_marker", 0);
        rvizMarkerPub_ = this->create_publisher<visualization_msgs::msg::Marker> ("visualization_marker", 0);

        cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(cam_image_topic, 1, 
              std::bind(&IndividualMarkersNoKinect::getCapCallback, this, std::placeholders::_1));


        RCLCPP_INFO(this->get_logger(),"Subscribing to info topic");
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(cam_info_topic, 1, std::bind(&IndividualMarkersNoKinect::InfoCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(1000ms, std::bind(&IndividualMarkersNoKinect::set_parameters, this));

    }


    void InfoCallback (const sensor_msgs::msg::CameraInfo::SharedPtr cam_info) 
    {
      if (!cam->getCamInfo_)
      {
          cam->SetCameraInfo(cam_info);
          cam->getCamInfo_ = true;
          info_sub_.reset();
      }
    }


    void set_parameters()
    {
        if (output_frame.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Param 'output_frame' has to be set. output frame %s",output_frame.c_str());
        }
        else
        {
          parameters_set=true;
        }

        this->get_parameter("marker_size", marker_size);
        this->get_parameter("max_new_marker_error", max_new_marker_error);
        this->get_parameter("max_track_error", max_track_error);
        this->get_parameter("max_frequency", max_frequency);
        this->get_parameter("marker_resolution", marker_resolution);
        this->get_parameter("marker_margin", marker_margin);
        this->get_parameter("output_frame", output_frame);
    }


    void getCapCallback (const sensor_msgs::msg::Image::SharedPtr image_msg)
    //void getCapCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
    {
        std::string tf_error;
        
        //If we've already gotten the cam info, then go ahead
        if(cam->getCamInfo_ && parameters_set){
		    try
        {
		      geometry_msgs::msg::TransformStamped CamToOutput; 
    			try
          {
            tf2::TimePoint tf2_time = tf2_ros::fromMsg(image_msg->header.stamp);
            CamToOutput = tf2_->lookupTransform(output_frame, image_msg->header.frame_id,tf2_time,tf2_time - prev_stamp_);
   				}
    			catch (tf2::TransformException ex){
      				RCLCPP_ERROR(this->get_logger(), "%s",ex.what());
              tf_error.clear();
    			}

          //Convert the image
          cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

          //Get the estimated pose of the main markers by using all the markers in each bundle
          // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
          // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
          // do this conversion here -jbinney

          cv::Mat ipl_image = cv_ptr_->image;

          marker_detector.Detect(cv_ptr_->image, cam, true, false,
                             max_new_marker_error, max_track_error, CVSEQ,
                             true);
          arPoseMarkers_.markers.clear();

          for (size_t i = 0; i < marker_detector.markers->size(); i++)
          {   
				    // Get the pose relative to the camera
            int id = (*(marker_detector.markers))[i].GetId();
            Pose p = (*(marker_detector.markers))[i].pose;
            double px = p.translation[0] / 100.0;
            double py = p.translation[1] / 100.0;
            double pz = p.translation[2] / 100.0;

            cv::Mat quat =cv::Mat(4, 1, CV_64F);
            p.GetQuaternion(quat);
            double qx = quat.at<double>(1,0); //p.quaternion[1]; #leaving these for the record, this was a bug in the original repo
            double qy = quat.at<double>(2,0); //p.quaternion[2];
            double qz = quat.at<double>(3,0); //p.quaternion[3];
            double qw = quat.at<double>(0,0); //p.quaternion[0];
            tf2::Quaternion rotation (qx,qy,qz,qw);
            tf2::Vector3 origin (px,py,pz);
            tf2::Transform t (rotation, origin);
            tf2::Vector3 markerOrigin (0, 0, 0);
            tf2::Transform m (tf2::Quaternion::getIdentity (), markerOrigin);
            tf2::Transform markerPose = t * m; // marker pose in the camera frame


            tf2::Vector3 z_axis_cam = tf2::Transform(rotation, tf2::Vector3(0,0,0)) * tf2::Vector3(0, 0, 1);

            /// as we can't see through markers, this one is false positive detection
            if (z_axis_cam.z() > 0)
            {
              continue;
            }

				    //Publish the transform from the camera to the marker
				    std::string markerFrame = "ar_marker_";
				    std::stringstream out;
				    out << id;
				    std::string id_string = out.str();
				    markerFrame += id_string;

            geometry_msgs::msg::TransformStamped camToMarker;
            camToMarker.header.stamp = image_msg->header.stamp;
            camToMarker.header.frame_id = image_msg->header.frame_id;
            camToMarker.child_frame_id = markerFrame;

            geometry_msgs::msg::Vector3 trans;
            trans.x = px;
            trans.y = py;
            trans.z = pz;
            geometry_msgs::msg::Quaternion rot;
            rot.x = qx;
            rot.y = qy;
            rot.z = qz;
            rot.w = qw;

            camToMarker.transform.translation = trans;
            camToMarker.transform.rotation = rot;
            tf_broadcaster_->sendTransform(camToMarker);
    		
            //Create the rviz visualization messages
            rvizMarker_.pose.position.x = markerPose.getOrigin().getX();
            rvizMarker_.pose.position.y = markerPose.getOrigin().getY();
            rvizMarker_.pose.position.z = markerPose.getOrigin().getZ();
            rvizMarker_.pose.orientation.x = markerPose.getRotation().getX();
            rvizMarker_.pose.orientation.y = markerPose.getRotation().getY();
            rvizMarker_.pose.orientation.z = markerPose.getRotation().getZ();
            rvizMarker_.pose.orientation.w = markerPose.getRotation().getW();
            rvizMarker_.header.frame_id = image_msg->header.frame_id;
            rvizMarker_.header.stamp = image_msg->header.stamp;
            rvizMarker_.id = id;

            rvizMarker_.scale.x = 1.0 * marker_size/100.0;
            rvizMarker_.scale.y = 1.0 * marker_size/100.0;
            rvizMarker_.scale.z = 0.2 * marker_size/100.0;
            rvizMarker_.ns = "basic_shapes";
            rvizMarker_.type = visualization_msgs::msg::Marker::CUBE;
            rvizMarker_.action = visualization_msgs::msg::Marker::ADD;
            switch (id)
            {
              case 0:
                rvizMarker_.color.r = 0.0f;
                rvizMarker_.color.g = 0.0f;
                rvizMarker_.color.b = 1.0f;
                rvizMarker_.color.a = 1.0;
                break;
              case 1:
                rvizMarker_.color.r = 1.0f;
                rvizMarker_.color.g = 0.0f;
                rvizMarker_.color.b = 0.0f;
                rvizMarker_.color.a = 1.0;
                break;
              case 2:
                rvizMarker_.color.r = 0.0f;
                rvizMarker_.color.g = 1.0f;
                rvizMarker_.color.b = 0.0f;
                rvizMarker_.color.a = 1.0;
                break;
              case 3:
                rvizMarker_.color.r = 0.0f;
                rvizMarker_.color.g = 0.5f;
                rvizMarker_.color.b = 0.5f;
                rvizMarker_.color.a = 1.0;
                break;
              case 4:
                rvizMarker_.color.r = 0.5f;
                rvizMarker_.color.g = 0.5f;
                rvizMarker_.color.b = 0.0;
                rvizMarker_.color.a = 1.0;
                break;
              default:
                rvizMarker_.color.r = 0.5f;
                rvizMarker_.color.g = 0.0f;
                rvizMarker_.color.b = 0.5f;
                rvizMarker_.color.a = 1.0;
                break;
            }
            rvizMarker_.lifetime = rclcpp::Duration (1.0);
            rvizMarkerPub_->publish (rvizMarker_);

            //Create the pose marker messages
            ar_track_alvar_msgs::msg::AlvarMarker ar_pose_marker;
            //Get the pose of the tag in the camera frame, then the output frame (usually torso)
            tf2::doTransform(ar_pose_marker.pose, ar_pose_marker.pose,CamToOutput);
            ar_pose_marker.header.frame_id = output_frame;
            ar_pose_marker.header.stamp = image_msg->header.stamp;
            ar_pose_marker.id = id;
            arPoseMarkers_.markers.push_back (ar_pose_marker);
			    }
			    arMarkerPub_->publish (arPoseMarkers_);
	  	  }
        catch (cv_bridge::Exception& e)
        {
      		RCLCPP_ERROR(this->get_logger(),"Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
    	  }
        prev_stamp_ = tf2_ros::fromMsg(image_msg->header.stamp);
	  }
  }

};

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndividualMarkersNoKinect>(argc,argv));
  rclcpp::shutdown();
  return 0;

}
