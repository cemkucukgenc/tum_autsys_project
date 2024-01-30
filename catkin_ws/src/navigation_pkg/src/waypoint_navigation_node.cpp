#include  "ros/ros.h"
#include <waypoint_navigation.h>

#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_planner");
    ros::NodeHandle n;
	
    BasicPlanner planner(n);  // instantiate basic planner
    ros::Duration(1.0).sleep();

    // define goal point
    Eigen::Vector3d goal_position, goal_velocity;
    goal_position << -321.0, 10.0, 15.0;
    goal_velocity << 0.0, 0.0, 0.0;

    for (int i = 0; i < 10; i++) {
        ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    }

    mav_trajectory_generation::Trajectory trajectory;
    planner.planTrajectory(goal_position, goal_velocity, &trajectory);
    planner.publishTrajectory(trajectory);
    ROS_WARN_STREAM("WAYPOINT GOAL REACHED!");
    return 0;
}
