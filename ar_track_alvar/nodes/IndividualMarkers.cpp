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


#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/msg/alvar_marker.hpp>
#include <ar_track_alvar_msgs/msg/alvar_markers.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/image_encodings.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/StdVector>
#include <memory>


using namespace alvar;
using namespace std;
using boost::make_shared;
namespace gm=geometry_msgs;
namespace ata=ar_track_alvar;
typedef pcl::PointXYZRGB ARPoint;
typedef pcl::PointCloud<ARPoint> ARCloud;


class IndividualMarkers : public rclcpp::Node
{
  private:
    bool init=true;
    Camera *cam;
    cv_bridge::CvImagePtr cv_ptr_;
  
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  cloud_sub_;
    rclcpp::Publisher<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr arMarkerPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rvizMarkerPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  rvizMarkerPub2_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;


    tf2::TimePoint prev_stamp_;
    std::shared_ptr<tf2_ros::Buffer> tf2_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
 

    double parameters_set = false;
    rclcpp::TimerBase::SharedPtr timer_;

    ar_track_alvar_msgs::msg::AlvarMarkers arPoseMarkers_;
    visualization_msgs::msg::Marker rvizMarker_;

    
    MarkerDetector<MarkerData> marker_detector;
    bool enableSwitched = false;
    bool output_frame_from_msg;
    double max_frequency;
    double marker_size;
    double max_new_marker_error;
    double max_track_error;
    std::string cam_image_topic;
    std::string cam_info_topic;
    std::string output_frame;
    int marker_resolution = 5; // default marker resolution
    int marker_margin = 2; // default marker margin




  public:
      IndividualMarkers(int argc, char* argv[]):Node("marker_detect")
      {

        rclcpp::Clock::SharedPtr clock = this->get_clock();
        tf2_ = std::make_shared<tf2_ros::Buffer>(clock);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        prev_stamp_ = tf2::get_now();

        this->declare_parameter<double>("marker_size", 10.0);
        this->declare_parameter<double>("max_new_marker_error", 0.08);
        this->declare_parameter<double>("max_track_error", 0.2);
        this->declare_parameter<double>("max_frequency", 8.0);
        this->declare_parameter<int>("marker_resolution", 5);
        this->declare_parameter<int>("marker_margin", 2);
        this->declare_parameter<bool>("output_frame_from_msg", false);
        this->declare_parameter<std::string>("output_frame", "");
          
        // Camera input topics. Use remapping to map to your camera topics.
        cam_image_topic = "camera_image";
        cam_info_topic = "camera_info";

        marker_detector.SetMarkerSize(marker_size, marker_resolution, marker_margin);
        cam = new Camera();


        arMarkerPub_ = this->create_publisher<ar_track_alvar_msgs::msg::AlvarMarkers> ("ar_pose_marker", 0);
        rvizMarkerPub_ = this->create_publisher<visualization_msgs::msg::Marker> ("visualization_marker", 0);
        rvizMarkerPub2_ = this->create_publisher<visualization_msgs::msg::Marker> ("ARmarker_points", 0);

      
        //Give tf a chance to catch up before the camera callback starts asking for transforms
        // It will also reconfigure parameters for the first time, setting the default values
        //TODO: come back to this, there's probably a better way to do this 
        rclcpp::Rate loop_rate(100);
        loop_rate.sleep();

        RCLCPP_INFO(this->get_logger(), "Subscribing to image topic");

        // subscribe to the point cloud
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cam_image_topic, 1, 
        std::bind(&IndividualMarkers::getPointCloudCallback, this, std::placeholders::_1));

        // subscribe to the info topic
        RCLCPP_INFO(this->get_logger(),"Subscribing to info topic");
	      info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(cam_info_topic, 1, std::bind(&IndividualMarkers::InfoCallback, this, std::placeholders::_1));

        // create parameter callback
        timer_ = this->create_wall_timer(1000ms, std::bind(&IndividualMarkers::set_parameters, this));
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
        if (!output_frame_from_msg && output_frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Param 'output_frame' has to be set if the output frame is not "
                      "derived from the point cloud message.");
            exit(0);
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
          this->get_parameter("output_frame_from_msg", output_frame_from_msg);
          this->get_parameter("output_frame", output_frame);
    }

  //Debugging utility function
    void draw3dPoints(ARCloud::Ptr cloud, string frame, int color, int id, double rad)
    {
      visualization_msgs::msg::Marker rvizMarker;

      rvizMarker.header.frame_id = frame;
      rvizMarker.header.stamp = this->get_clock()->now();
      rvizMarker.id = id;
      rvizMarker.ns = "3dpts";

      rvizMarker.scale.x = rad;
      rvizMarker.scale.y = rad;
      rvizMarker.scale.z = rad;

      rvizMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      rvizMarker.action = visualization_msgs::msg::Marker::ADD;

      if(color==1){
        rvizMarker.color.r = 0.0f;
        rvizMarker.color.g = 1.0f;
        rvizMarker.color.b = 1.0f;
        rvizMarker.color.a = 1.0;
      }
      if(color==2){
        rvizMarker.color.r = 1.0f;
        rvizMarker.color.g = 0.0f;
        rvizMarker.color.b = 1.0f;
        rvizMarker.color.a = 1.0;
      }
      if(color==3){
        rvizMarker.color.r = 1.0f;
        rvizMarker.color.g = 1.0f;
        rvizMarker.color.b = 0.0f;
        rvizMarker.color.a = 1.0;
      }

      gm::msg::Point p;
      for(int i=0; i<cloud->points.size(); i++){
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        rvizMarker.points.push_back(p);
      }

      rvizMarker.lifetime = rclcpp::Duration (1.0);
      rvizMarkerPub2_->publish (rvizMarker);
    }


      void drawArrow(gm::msg::Point start, tf2::Matrix3x3 mat, string frame, int color, int id)
      {
        visualization_msgs::msg::Marker rvizMarker;

        rvizMarker.header.frame_id = frame;
        rvizMarker.header.stamp = this->get_clock()->now();
        rvizMarker.id = id;
        rvizMarker.ns = "arrow";

        rvizMarker.scale.x = 0.01;
        rvizMarker.scale.y = 0.01;
        rvizMarker.scale.z = 0.1;

        rvizMarker.type = visualization_msgs::msg::Marker::ARROW;
        rvizMarker.action = visualization_msgs::msg::Marker::ADD;

        for(int i=0; i<3; i++){
          rvizMarker.points.clear();
          rvizMarker.points.push_back(start);
          gm::msg::Point end;
          end.x = start.x + mat[0][i];
          end.y = start.y + mat[1][i];
          end.z = start.z + mat[2][i];
          rvizMarker.points.push_back(end);
          rvizMarker.id += 10*i;
          rvizMarker.lifetime = rclcpp::Duration (1.0);

          if(color==1){
            rvizMarker.color.r = 1.0f;
            rvizMarker.color.g = 0.0f;
            rvizMarker.color.b = 0.0f;
            rvizMarker.color.a = 1.0;
          }
          if(color==2){
            rvizMarker.color.r = 0.0f;
            rvizMarker.color.g = 1.0f;
            rvizMarker.color.b = 0.0f;
            rvizMarker.color.a = 1.0;
          }
          if(color==3){
            rvizMarker.color.r = 0.0f;
            rvizMarker.color.g = 0.0f;
            rvizMarker.color.b = 1.0f;
            rvizMarker.color.a = 1.0;
          }
          color += 1;

          rvizMarkerPub2_->publish (rvizMarker);
        }
      }

      int PlaneFitPoseImprovement(int id, const ARCloud& corners_3D,
                                ARCloud::Ptr selected_points, const ARCloud& cloud,
                                Pose& p)
    {
      ata::PlaneFitResult res = ata::fitPlane(selected_points);
      gm::msg::PoseStamped pose;
      pose.header.stamp = pcl_conversions::fromPCL(cloud.header).stamp;
      pose.header.frame_id = cloud.header.frame_id;
      pose.pose.position = ata::centroid(*res.inliers);

      draw3dPoints(selected_points, cloud.header.frame_id, 1, id, 0.005);

      // Get 2 points that point forward in marker x direction
      int i1, i2;
      if (isnan(corners_3D[0].x) || isnan(corners_3D[0].y) ||
          isnan(corners_3D[0].z) || isnan(corners_3D[3].x) ||
          isnan(corners_3D[3].y) || isnan(corners_3D[3].z))
      {
        if (isnan(corners_3D[1].x) || isnan(corners_3D[1].y) ||
            isnan(corners_3D[1].z) || isnan(corners_3D[2].x) ||
            isnan(corners_3D[2].y) || isnan(corners_3D[2].z))
        {
          return -1;
        }
        else
        {
          i1 = 1;
          i2 = 2;
        }
      }
      else
      {
        i1 = 0;
        i2 = 3;
      }

      // Get 2 points the point forward in marker y direction
      int i3, i4;
      if (isnan(corners_3D[0].x) || isnan(corners_3D[0].y) ||
          isnan(corners_3D[0].z) || isnan(corners_3D[1].x) ||
          isnan(corners_3D[1].y) || isnan(corners_3D[1].z))
      {
        if (isnan(corners_3D[3].x) || isnan(corners_3D[3].y) ||
            isnan(corners_3D[3].z) || isnan(corners_3D[2].x) ||
            isnan(corners_3D[2].y) || isnan(corners_3D[2].z))
        {
          return -1;
        }
        else
        {
          i3 = 2;
          i4 = 3;
        }
      }
      else
      {
        i3 = 1;
        i4 = 0;
      }

      ARCloud::Ptr orient_points(new ARCloud());
      orient_points->points.push_back(corners_3D[i1]);
      draw3dPoints(orient_points, cloud.header.frame_id, 3, id + 1000, 0.008);

      orient_points->clear();
      orient_points->points.push_back(corners_3D[i2]);
      draw3dPoints(orient_points, cloud.header.frame_id, 2, id + 2000, 0.008);

      int succ;
      succ = ata::extractOrientation(res.coeffs, corners_3D[i1], corners_3D[i2],
                                    corners_3D[i3], corners_3D[i4],
                                    pose.pose.orientation);
      if (succ < 0)
        return -1;

      tf2::Matrix3x3 mat;
      succ = ata::extractFrame(res.coeffs, corners_3D[i1], corners_3D[i2],
                              corners_3D[i3], corners_3D[i4], mat);
      if (succ < 0)
        return -1;

      drawArrow(pose.pose.position, mat, cloud.header.frame_id, 1, id);

      p.translation[0] = pose.pose.position.x * 100.0;
      p.translation[1] = pose.pose.position.y * 100.0;
      p.translation[2] = pose.pose.position.z * 100.0;
      p.quaternion[1] = pose.pose.orientation.x;
      p.quaternion[2] = pose.pose.orientation.y;
      p.quaternion[3] = pose.pose.orientation.z;
      p.quaternion[0] = pose.pose.orientation.w;

      return 0;
    }

      void GetMarkerPoses(cv::Mat * image, ARCloud &cloud) 
      {

        //Detect and track the markers
        if (marker_detector.Detect(*image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true))
        {
        
        RCLCPP_INFO(this->get_logger(), "-----------------------------------");
        for (size_t i=0; i<marker_detector.markers->size(); i++)
        {
            vector<cv::Point, Eigen::aligned_allocator<cv::Point> > pixels;
            Marker *m = &((*marker_detector.markers)[i]);
            int id = m->GetId();

            RCLCPP_INFO(this->get_logger(), "******* ID: %d",id);


            int resol = m->GetRes();
            int ori = m->ros_orientation;

            PointDouble pt1, pt2, pt3, pt4;
            pt4 = m->ros_marker_points_img[0];
            pt3 = m->ros_marker_points_img[resol-1];
            pt1 = m->ros_marker_points_img[(resol*resol)-resol];
            pt2 = m->ros_marker_points_img[(resol*resol)-1];

            m->ros_corners_3D[0] = cloud(pt1.x, pt1.y);
            m->ros_corners_3D[1] = cloud(pt2.x, pt2.y);
            m->ros_corners_3D[2] = cloud(pt3.x, pt3.y);
            m->ros_corners_3D[3] = cloud(pt4.x, pt4.y);

            if(ori >= 0 && ori < 4)
            {
              if(ori != 0)
              {
                std::rotate(m->ros_corners_3D.begin(), m->ros_corners_3D.begin() + ori, m->ros_corners_3D.end());
              }
            }
            else
            {
              RCLCPP_ERROR(this->get_logger(), "FindMarkerBundles: Bad Orientation: %i for ID: %i", ori, id);

              //Get the 3D marker points
              BOOST_FOREACH (const PointDouble& p, m->ros_marker_points_img)
              {
                pixels.push_back(cv::Point(p.x, p.y));
              }
              ARCloud::Ptr selected_points = ata::filterCloud(cloud, pixels);

              //Use the kinect data to find a plane and pose for the marker
              int ret = PlaneFitPoseImprovement(i, m->ros_corners_3D, selected_points, cloud, m->pose);
            }
          }
        }}

      void getPointCloudCallback (const sensor_msgs::msg::PointCloud2::SharedPtr msg)
      {
        sensor_msgs::msg::Image image_msg;

        // If desired, use the frame in the message's header.
        if (output_frame_from_msg)
          output_frame = msg->header.frame_id;

        //If we've already gotten the cam info, then go ahead
        if(cam->getCamInfo_){
          //Convert cloud to PCL
          ARCloud cloud;
          pcl::fromROSMsg(*msg, cloud);
          //Get an OpenCV image from the cloud
          pcl::toROSMsg (*msg, image_msg);


          image_msg.header.stamp = msg->header.stamp;
          image_msg.header.frame_id = msg->header.frame_id;


          //Convert the image
          cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

          //Get the estimated pose of the main markers by using all the markers in each bundle

          // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
          // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
          // do this conversion here -jbinney
          cv::Mat ipl_image = cv_ptr_->image;


          //Use the kinect to improve the pose
          Pose ret_pose;
          GetMarkerPoses(&ipl_image, cloud);
          std::string tf_error;
          geometry_msgs::msg::TransformStamped CamToOutput;
          try 
          {
              tf2::TimePoint tf2_time = tf2_ros::fromMsg(image_msg.header.stamp);
              CamToOutput = tf2_->lookupTransform(output_frame, image_msg.header.frame_id,tf2_time,tf2_time - prev_stamp_);

          } catch (tf2::TransformException ex) {
              RCLCPP_ERROR(this->get_logger(), "%s",ex.what());
          }

        try
        {
          arPoseMarkers_.markers.clear ();
          for (size_t i=0; i<marker_detector.markers->size(); i++)
          {
            //Get the pose relative to the camera
            int id = (*(marker_detector.markers))[i].GetId();
            Pose p = (*(marker_detector.markers))[i].pose;

            double px = p.translation[0]/100.0;
            double py = p.translation[1]/100.0;
            double pz = p.translation[2]/100.0;
            double qx = p.quaternion[1];
            double qy = p.quaternion[2];
            double qz = p.quaternion[3];
            double qw = p.quaternion[0];

            tf2::Quaternion rotation (qx,qy,qz,qw);
            tf2::Vector3 origin (px,py,pz);
            tf2::Transform t (rotation, origin);
            tf2::Vector3 markerOrigin (0, 0, 0);
            tf2::Transform m (tf2::Quaternion::getIdentity (), markerOrigin);
            tf2::Transform markerPose = t * m; // marker pose in the camera frame

            //Publish the transform from the camera to the marker
            std::string markerFrame = "ar_marker_";
            std::stringstream out;
            out << id;
            std::string id_string = out.str();
            markerFrame += id_string;
            geometry_msgs::msg::TransformStamped camToMarker;
            camToMarker.header.stamp = image_msg.header.stamp;
            camToMarker.header.frame_id = image_msg.header.frame_id;
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

            rvizMarker_.header.frame_id = image_msg.header.frame_id;
            rvizMarker_.header.stamp = image_msg.header.stamp;
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

          //Get the pose of the tag in the camera frame, then the output frame (usually torso)



          // tf2::Transform tagPoseOutput = CamToOutput * markerPose;
          //Create the pose marker messages
          ar_track_alvar_msgs::msg::AlvarMarker ar_pose_marker;
          tf2::doTransform(ar_pose_marker.pose, ar_pose_marker.pose,CamToOutput);
          ar_pose_marker.header.frame_id = output_frame;
          ar_pose_marker.header.stamp = image_msg.header.stamp;
          ar_pose_marker.id = id;
          arPoseMarkers_.markers.push_back (ar_pose_marker);
        }
            arPoseMarkers_.header.stamp = image_msg.header.stamp;
            arMarkerPub_->publish (arPoseMarkers_);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"Could not convert from '%s' to 'rgb8'.", image_msg.encoding.c_str ());
        }
        prev_stamp_ = tf2_ros::fromMsg(image_msg.header.stamp);
      }
      }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndividualMarkers>(argc,argv));
  rclcpp::shutdown();
  return 0;

}
