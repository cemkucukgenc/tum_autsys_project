#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <depth_image_proc/depth_traits.h>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <limits>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

class LightDetectorNode {
  ros::NodeHandle nodeHandle_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber semantic_image_subscriber_;
  image_transport::Subscriber depth_image_subscriber_;
  ros::Subscriber depth_info_subscriber_;

  ros::Publisher marker_publisher_;
  ros::Publisher point_cloud_publisher_;

  public:
  LightDetectorNode() : image_transport_(nodeHandle_) {
    semantic_image_subscriber_ = image_transport_.subscribe("/realsense/semantic/image_raw", 5, &LightDetectorNode::onSemanticImageReceived, this);
    depth_image_subscriber_ = image_transport_.subscribe("/realsense/depth/image", 5, &LightDetectorNode::onDepthImageReceived, this);
    depth_info_subscriber_ = nodeHandle_.subscribe("/realsense/depth/camera_info", 5, &LightDetectorNode::onDepthInfoReceived, this);

    marker_publisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("detected_objects_markers", 10);
    point_cloud_publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("detected_objects_point_cloud", 10);
  }
  void onSemanticImageReceived(const sensor_msgs::ImageConstPtr& semantic_image_msg) {
    // TODO
  }

  void onDepthImageReceived(const sensor_msgs::ImageConstPtr& depth_image_msg) {
    // TODO
  }

  void onDepthInfoReceived(const sensor_msgs::CameraInfo& depth_info_msg) {
    // TODO
  }
  void publishMarkers(const pcl::PointXYZ det, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = det.x;
    marker.pose.position.y = det.y;
    marker.pose.position.z = det.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2;
    marker.scale.y = 2;
    marker.scale.z = 4;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_publisher_.publish( marker );
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "light_detector_node");
  LightDetectorNode node;
  ros::spin();
}