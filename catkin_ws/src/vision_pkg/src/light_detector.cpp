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
  ros::NodeHandle nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber semantic_image_subscriber_;
  image_transport::Subscriber depth_image_subscriber_;
  ros::Subscriber depth_info_subscriber_;

  tf::TransformListener transform_listener_;

  std::vector<pcl::PointXYZ> detections_;

  sensor_msgs::Image last_depth_image_;
  sensor_msgs::CameraInfo last_depth_info_;
  image_geometry::PinholeCameraModel camera_model_;

public:
  LightDetectorNode() : image_transport_(nh_) {
    semantic_image_subscriber_ = image_transport_.subscribe(
        "/realsense/semantic/image_raw", 5,
        &LightDetectorNode::onSemanticImageReceived, this);
    depth_image_subscriber_ = image_transport_.subscribe(
        "/realsense/depth/image", 5, &LightDetectorNode::onDepthImageReceived,
        this);
    depth_info_subscriber_ =
        nh_.subscribe("/realsense/depth/camera_info", 5,
                      &LightDetectorNode::onDepthInfoReceived, this);
  }
  void onSemanticImageReceived(
      const sensor_msgs::ImageConstPtr &semantic_image_msg) {
    auto masked_depth_image = extractDepthImageWithMask(semantic_image_msg);
  }

  void onDepthImageReceived(const sensor_msgs::ImageConstPtr &depth_image_msg) {
    // TODO
  }

  void onDepthInfoReceived(const sensor_msgs::CameraInfo &depth_info_msg) {
    // TODO
  }

private:
  cv::Mat extractDepthImageWithMask(
      const sensor_msgs::ImageConstPtr &semantic_image_msg) {
    auto semantic_image = cv_bridge::toCvCopy(
        semantic_image_msg, sensor_msgs::image_encodings::BGR8);
    auto depth_image = cv_bridge::toCvCopy(
        last_depth_image_, sensor_msgs::image_encodings::TYPE_16UC1);

    cv::Mat mask;
    cv::inRange(semantic_image->image, cv::Scalar(4, 235, 255),
                cv::Scalar(4, 235, 255), mask);
    depth_image->image.setTo(
        cv::Scalar(std::numeric_limits<double>::quiet_NaN()), ~mask);

    return depth_image->image;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "light_detector_node");
  LightDetectorNode node;
  ros::spin();
}