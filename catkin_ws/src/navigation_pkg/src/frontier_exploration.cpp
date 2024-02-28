#include "Optics.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <vector>

// Hypothetical GitHub repository URLs remain unchanged
// Source for Octomap integration: https://github.com/OctoMap/octomap_ros
// Source for the OPTICS clustering algorithm:
// https://github.com/Nandite/Pcl-Optics/tree/master

class Frontier {
public:
  Frontier();

private:
  ros::NodeHandle node_handle;
  ros::Subscriber state_subscriber, map_subscriber;
  ros::Publisher goal_publisher;
  ros::Timer exploration_timer;

  octomap::OcTree::leaf_bbx_iterator iterator;
  std::shared_ptr<octomap::OcTree> octree{};
  octomap::point3d current_position;
  std::vector<geometry_msgs::Point> frontier_points;
  int octomap_resolution, max_distance; // Example max distance
  std::vector<pcl::PointIndicesPtr> cluster_indices;

  void currentStateCallback(const nav_msgs::Odometry &msg);
  void mapUpdateCallback(const octomap_msgs::OctomapConstPtr &msg);
  void explorationTimerCallback(const ros::TimerEvent &event);

  pcl::PointCloud<pcl::PointXYZ>::Ptr generateFrontierCloud();
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  identifyLargestCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  pcl::PointXYZ calculateGoal(pcl::PointCloud<pcl::PointXYZ> &cloud);
  void detectFrontiers();
  bool isFrontierPoint(const octomap::point3d &coord);
  void publishGoal(const pcl::PointXYZ &goal);
};

Frontier::Frontier() {
  state_subscriber = node_handle.subscribe(
      "current_state_est", 1, &Frontier::currentStateCallback, this);
  map_subscriber = node_handle.subscribe("octomap_full", 1,
                                         &Frontier::mapUpdateCallback, this);
  goal_publisher =
      node_handle.advertise<geometry_msgs::Point>("frontier_goal", 1);

  node_handle.getParam("/octomap_server/resolution", octomap_resolution);
  exploration_timer = node_handle.createTimer(
      ros::Rate(0.3), &Frontier::explorationTimerCallback, this);
}

void Frontier::currentStateCallback(const nav_msgs::Odometry &msg) {
  current_position =
      octomap::point3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                       msg.pose.pose.position.z);
}

void Frontier::mapUpdateCallback(const octomap_msgs::OctomapConstPtr &msg) {
  octree = std::make_shared<octomap::OcTree>(
      *dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
}

void Frontier::explorationTimerCallback(const ros::TimerEvent &event) {
  if (!octree)
    return;

  detectFrontiers();
  auto frontier_cloud = generateFrontierCloud();

  // Assuming Optics::optics method exists and works as expected
  Optics::optics<pcl::PointXYZ>(frontier_cloud, 5, 10.0, cluster_indices);

  auto largest_cluster_cloud = identifyLargestCluster(frontier_cloud);
  auto goal = calculateGoal(*largest_cluster_cloud);

  ROS_INFO("Frontier goal point set to: (%f, %f, %f)", goal.x, goal.y, goal.z);
  publishGoal(goal);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Frontier::generateFrontierCloud() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>); // Use boost::shared_ptr
  cloud->header.frame_id = "world";
  cloud->is_dense = false;

  for (const auto &point : frontier_points) {
    cloud->points.emplace_back(point.x, point.y, point.z);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
Frontier::identifyLargestCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // TODO
}
pcl::PointXYZ Frontier::calculateGoal(pcl::PointCloud<pcl::PointXYZ> &cloud) {
  // TODO
}

void Frontier::detectFrontiers() {
  // TODO
}

bool Frontier::isFrontierPoint(const octomap::point3d &coord) {
  // Define the search offsets to check the immediate neighbors in 3D,
  // including diagonals
  std::vector<octomap::point3d> searchOffsets =
      {{1, 0, 0},   {-1, 0, 0},   {0, 1, 0},
       {0, -1, 0},  {0, 0, 1},    {0, 0, -1},
       {1, 1, 0},   {-1, -1, 0},  {1, -1, 0},
       {-1, 1, 0}, // Diagonals in XY plane
       {1, 0, 1},   {-1, 0, -1},  {0, 1, 1},
       {0, -1, -1}, // Diagonals in XZ and YZ planes
       {-1, 0, 1},  {1, 0, -1},   {0, -1, 1},
       {0, 1, -1}, // Opposite diagonals
       {1, 1, 1},   {-1, -1, -1}, {1, -1, 1},
       {-1, 1, -1}, // 3D diagonals
       {1, 1, -1},  {-1, -1, 1},  {1, -1, -1},
       {-1, 1, 1}};

  for (const auto &offset : searchOffsets) {
    octomap::point3d neighborCoord =
        coord + offset * octomap_resolution; // Adjust by octomap resolution
    octomap::OcTreeNode *node = octree->search(neighborCoord);
    if (!node) { // If the node does not exist (is unknown)
      return true;
    }
  }
  return false; // If all neighbors are known (occupied or free), it's not a
                // frontier
}

void Frontier::publishGoal(const pcl::PointXYZ &goal) {
  geometry_msgs::Point goal_msg;
  goal_msg.x = goal.x;
  goal_msg.y = goal.y;
  goal_msg.z = goal.z;
  goal_publisher.publish(goal_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "frontier_exploration");
  Frontier frontier_exploration;

  ros::spin();
  return 0;
}
