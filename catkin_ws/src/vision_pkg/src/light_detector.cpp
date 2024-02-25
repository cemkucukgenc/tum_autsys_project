#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <depth_image_proc/depth_traits.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <geometry_msgs/PointStamped.h>

class LightDetectorNode {
  ros::NodeHandle nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber semantic_image_subscriber_;
  image_transport::Subscriber depth_image_subscriber_;
  ros::Subscriber depth_info_subscriber_;

  tf::TransformListener transform_listener_;

  std::vector<pcl::PointXYZ> detections_;
  ros::Publisher detected_points_publisher_;

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

    detected_points_publisher_ =
        nh_.advertise<geometry_msgs::PointStamped>("detected_points", 10);
  }
  void onSemanticImageReceived(
      const sensor_msgs::ImageConstPtr &semantic_image_msg) {
    auto masked_depth_image = extractDepthImageWithMask(semantic_image_msg);

    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
    createPointCloudFromDepthImage(cloud_msg, masked_depth_image);

    auto world_frame_cloud = transformPointCloudToGlobalFrame(cloud_msg);
    if (world_frame_cloud.points.empty()) {
      return;
    }
    auto detection_center = calculatePointCloudCenter(world_frame_cloud);

    // Check if detection_center coordinates are not NaN before processing
    if (!std::isnan(detection_center.x) && !std::isnan(detection_center.y) &&
        !std::isnan(detection_center.z)) {
      if (checkForNewDetection(detection_center)) {
        ROS_WARN("Light detected at %f, %f, %f", detection_center.x,
                 detection_center.y, detection_center.z);
        detections_.push_back(detection_center);

        // Create and publish the PointStamped message
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now();
        point_msg.header.frame_id = "world"; // or the appropriate frame
        point_msg.point.x = detection_center.x;
        point_msg.point.y = detection_center.y;
        point_msg.point.z = detection_center.z;
        detected_points_publisher_.publish(point_msg);
      }
    }
  }

  void onDepthImageReceived(const sensor_msgs::ImageConstPtr &depth_image_msg) {
    last_depth_image_ = *depth_image_msg;
  }

  void onDepthInfoReceived(const sensor_msgs::CameraInfo &depth_info_msg) {
    last_depth_info_ = depth_info_msg;
    camera_model_.fromCameraInfo(depth_info_msg);
  }

private:
  void createPointCloudFromDepthImage(sensor_msgs::PointCloud2::Ptr &cloud_msg,
                                      const cv::Mat &depth_image) {
    cloud_msg->header = last_depth_image_.header;
    cloud_msg->height = last_depth_image_.height;
    cloud_msg->width = last_depth_image_.width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier cloud_modifier(*cloud_msg);
    cloud_modifier.setPointCloud2FieldsByString(1, "xyz");

    depthImageToPointCloud(depth_image, cloud_msg, camera_model_);
  }

  pcl::PointCloud<pcl::PointXYZ>
  transformPointCloudToGlobalFrame(sensor_msgs::PointCloud2::Ptr cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ> local_cloud;
    pcl::fromROSMsg(*cloud_msg, local_cloud);

    tf::StampedTransform transform;
    try {
      transform_listener_.waitForTransform("world", cloud_msg->header.frame_id,
                                           ros::Time(0), ros::Duration(10.0));
      transform_listener_.lookupTransform("world", cloud_msg->header.frame_id,
                                          ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
    }

    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    pcl_ros::transformPointCloud(local_cloud, global_cloud, transform);
    return global_cloud;
  }

  pcl::PointXYZ
  calculatePointCloudCenter(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    pcl::PointXYZ center(0, 0, 0); // Initialize the center point
    int valid_points = 0;
    for (const auto &point : cloud.points) {
      // Check if point is valid and not equal to (0, 0, 0)
      if (!std::isnan(point.x) && !std::isnan(point.y) &&
          !std::isnan(point.z) &&
          (point.x != 0 || point.y != 0 || point.z != 0)) {
        center.x += point.x;
        center.y += point.y;
        center.z += point.z;
        valid_points++;
      }
    }
    if (valid_points > 0) {
      center.x /= valid_points;
      center.y /= valid_points;
      center.z /= valid_points;
    } else {
      // Set the center to NaN to indicate invalid/missing data
      center.x = std::numeric_limits<float>::quiet_NaN();
      center.y = std::numeric_limits<float>::quiet_NaN();
      center.z = std::numeric_limits<float>::quiet_NaN();
    }
    return center;
  }

  void
  depthImageToPointCloud(const cv::Mat &depth_image,
                         sensor_msgs::PointCloud2::Ptr &cloud_msg,
                         const image_geometry::PinholeCameraModel &camera_model,
                         double range_max = 0.0) {
    float center_x = camera_model.cx();
    float center_y = camera_model.cy();

    double unit_scaling =
        depth_image_proc::DepthTraits<uint16_t>::toMeters(uint16_t(1));
    float constant_x = unit_scaling / camera_model.fx();
    float constant_y = unit_scaling / camera_model.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

    const uint16_t *depth_row =
        reinterpret_cast<const uint16_t *>(depth_image.data);
    int row_step = depth_image.step / sizeof(uint16_t);

    for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step) {
      for (int u = 0; u < (int)cloud_msg->width;
           ++u, ++iter_x, ++iter_y, ++iter_z) {
        uint16_t depth = depth_row[u];

        if (!depth_image_proc::DepthTraits<uint16_t>::valid(depth)) {
          if (range_max != 0.0) {
            depth =
                depth_image_proc::DepthTraits<uint16_t>::fromMeters(range_max);
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

  bool checkForNewDetection(const pcl::PointXYZ &point) {
    for (const auto &detection : detections_) {
      double distance_squared = std::pow(detection.x - point.x, 2) +
                                std::pow(detection.y - point.y, 2) +
                                std::pow(detection.z - point.z, 2);
      if (distance_squared < 100) {
        return false;
      }
    }
    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "light_detector_node");
  LightDetectorNode node;
  ros::spin();
}