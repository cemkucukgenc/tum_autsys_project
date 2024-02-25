#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <depth_image_proc/depth_traits.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

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

    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
    createPointCloudFromDepthImage(cloud_msg, masked_depth_image);
  }

  void onDepthImageReceived(const sensor_msgs::ImageConstPtr &depth_image_msg) {
    // TODO
  }

  void onDepthInfoReceived(const sensor_msgs::CameraInfo &depth_info_msg) {
    // TODO
  }

private:

void createPointCloudFromDepthImage(sensor_msgs::PointCloud2::Ptr& cloud_msg, const cv::Mat& depth_image) {
    cloud_msg->header = last_depth_image_.header;
    cloud_msg->height = last_depth_image_.height;
    cloud_msg->width = last_depth_image_.width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier cloud_modifier(*cloud_msg);
    cloud_modifier.setPointCloud2FieldsByString(1, "xyz");

    depthImageToPointCloud(depth_image, cloud_msg, camera_model_);
  }

  void depthImageToPointCloud(
    const cv::Mat& depth_image,
    sensor_msgs::PointCloud2::Ptr& cloud_msg,
    const image_geometry::PinholeCameraModel& camera_model,
    double range_max = 0.0) {
    float center_x = camera_model.cx();
    float center_y = camera_model.cy();

    double unit_scaling = depth_image_proc::DepthTraits<uint16_t>::toMeters(uint16_t(1));
    float constant_x = unit_scaling / camera_model.fx();
    float constant_y = unit_scaling / camera_model.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

    const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(depth_image.data);
    int row_step = depth_image.step / sizeof(uint16_t);

    for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step) {
      for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z) {
        uint16_t depth = depth_row[u];

        if (!depth_image_proc::DepthTraits<uint16_t>::valid(depth)) {
          if (range_max != 0.0) {
            depth = depth_image_proc::DepthTraits<uint16_t>::fromMeters(range_max);
          } else {
            *iter_x = *iter_y = *iter_z = bad_point;
            continue;
          }
        }

        *iter_x = (u - center_x) * depth * constant_x;
        *iter_y = (v - center_y) * depth * constant_y;
        *iter_z = depth_image_proc::DepthTraits<uint16_t>::toMeters(depth);
      }
    }
  }
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