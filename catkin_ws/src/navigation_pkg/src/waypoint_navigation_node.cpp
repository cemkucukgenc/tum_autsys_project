#include "ros/ros.h"
#include <waypoint_navigation.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <Eigen/Dense>

// Global pointer to BasicPlanner
BasicPlanner* planner = nullptr;
Eigen::Vector3d goal_velocity;
Eigen::Vector3d last_goal_position = Eigen::Vector3d::Zero(); // Initialize with zero

// Function to check if two positions are different
bool isDifferentPosition(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2) {
    // Define a small threshold for position differences
    const double threshold = 0.01; // Adjust this threshold as needed
    return (pos1 - pos2).norm() > threshold;
}

// Callback function for goal position updates
void goalCallback(const geometry_msgs::Point::ConstPtr& msg) {
    Eigen::Vector3d goal_position = Eigen::Vector3d(msg->x, msg->y, msg->z);

    if (isDifferentPosition(goal_position, last_goal_position)) {
        ROS_INFO_STREAM("Received new goal position: " << goal_position.transpose());

        mav_trajectory_generation::Trajectory trajectory;
        planner->planTrajectory(goal_position, goal_velocity, &trajectory);
        planner->publishTrajectory(trajectory);

        last_goal_position = goal_position; // Update the last goal position
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_planner");  // Initialize ROS
    ros::NodeHandle n;  // Create a NodeHandle after ROS has been initialized

    // Dynamically allocate the planner
    planner = new BasicPlanner(n);
    goal_velocity << 0.0, 0.0, 0.0;

    ros::Subscriber goal_sub = n.subscribe("goal_position", 1, goalCallback);

    ros::spin();  // Keep the node running and listening to callbacks

    // Clean up
    delete planner;
    return 0;
}

