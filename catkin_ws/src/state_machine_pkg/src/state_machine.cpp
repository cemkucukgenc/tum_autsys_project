#include <state_machine.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <cstdlib>
#include <iostream>

StateMachine::StateMachine() : waypoint_navigation_launched(false)
{
    double x, y, z;
    x = -38.0;
    y = 10.0;
    z = 6.9;

    addGoalPoint(-38.0, 10.0, 10.0); // take off from initial position
    addGoalPoint(-55, 0.84, 15.0);   // first lamp position at the outside
    addGoalPoint(-321, 10.0, 15.0);  // cave entrance
    addGoalPoint(-500, 0.0, 10.0);
    addGoalPoint(-599, -9.0, 10.0);
    addGoalPoint(-599, -5, 10.0);
    addGoalPoint(-599, -2, 3.0);

    origin_ = tf::Vector3(x, y, z);

    desired_state_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);
    current_state_sub_ = nh.subscribe("current_state_est", 1, &StateMachine::onCurrentState, this);
    goal_position_pub_ = nh.advertise<geometry_msgs::PoseStamped>("goal_position", 1);
    path_sub_ = nh.subscribe("planned_path", 1, &StateMachine::onPlannedPath, this);

    state_machine_timer_ = nh.createTimer(ros::Duration(sim_interval_),
                                          &StateMachine::state_machine_mission, this);
}

geometry_msgs::Point StateMachine::getNextGoalPoint()
{
    if (current_goal_index < goalpoints.size())
    {
        return goalpoints[current_goal_index++];
    }
    else
    {
        // Return the last point or handle the end of the vector as needed
        return goalpoints.back();
    }
}

void StateMachine::onPlannedPath(const nav_msgs::Path::ConstPtr &msg)
{
    // Resetting the path vector for new data
    goalpoints_path_vec.clear();
    auto received_poses = msg->poses;

    // Ensuring the path has at least three points by adding a midpoint if needed
    if (received_poses.size() == 2)
    {
        // Correct initialization of Eigen::Vector3d
        Eigen::Vector3d first_point(
            received_poses[0].pose.position.x,
            received_poses[0].pose.position.y,
            received_poses[0].pose.position.z);
        Eigen::Vector3d second_point(
            received_poses[1].pose.position.x,
            received_poses[1].pose.position.y,
            received_poses[1].pose.position.z);
        Eigen::Vector3d mid_point = (first_point + second_point) * 0.5;

        geometry_msgs::PoseStamped new_pose;
        new_pose.pose.position.x = mid_point(0); // Access elements with ()
        new_pose.pose.position.y = mid_point(1);
        new_pose.pose.position.z = mid_point(2);
        received_poses.insert(received_poses.begin() + 1, new_pose); // Use vector's insert method correctly
    }
    // Computing and appending orientation data to each path point
    for (size_t index = 0; index < received_poses.size(); ++index)
    {
        // Decomposing the current pose to basic coordinates
        auto current_pose = received_poses[index];
        double posX = current_pose.pose.position.x;
        double posY = current_pose.pose.position.y;
        double posZ = current_pose.pose.position.z;
        double orientation_yaw;
        Eigen::Vector3d current_position(posX, posY, posZ), next_position;
        size_t index_next = index + 1;
        if (index_next < received_poses.size())
        {
            double nextPosX = received_poses[index_next].pose.position.x;
            double nextPosY = received_poses[index_next].pose.position.y;
            double nextPosZ = received_poses[index_next].pose.position.z;
            next_position = Eigen::Vector3d(nextPosX, nextPosY, nextPosZ);

            Eigen::Vector3d direction_vector = next_position - current_position;
            orientation_yaw = std::atan2(direction_vector[1], direction_vector[0]);
        }
        else
        {
            orientation_yaw = goalpoints_path_vec[index - 1][3]; // Reuse the previous yaw for the last point
        }

        // Storing the processed data in a 4D vector
        goalpoints_path_vec.push_back(Eigen::Vector4d(posX, posY, posZ, orientation_yaw));
    }
    // Transform the last goalpoint to goal_pose_stamped
    if (!goalpoints_path_vec.empty())
    {
        const Eigen::Vector4d &goalPoint = goalpoints_path_vec.back();
        geometry_msgs::PoseStamped goal_pose_stamped;
        goal_pose_stamped.pose.position.x = goalPoint[0];
        goal_pose_stamped.pose.position.y = goalPoint[1];
        goal_pose_stamped.pose.position.z = goalPoint[2];

        tf2::Quaternion q;
        q.setRPY(0, 0, goalPoint[3]); // roll, pitch, yaw

        // Convert tf2 quaternion to geometry_msgs quaternion
        goal_pose_stamped.pose.orientation = tf2::toMsg(q);

        // Set the header
        goal_pose_stamped.header.frame_id = "world"; // Set this to your frame id
        goal_pose_stamped.header.stamp = ros::Time::now();

        // Publish the goal_pose_stamped
        goal_position_pub_.publish(goal_pose_stamped);
    }
}

geometry_msgs::PoseStamped StateMachine::pointToPoseStamped(const geometry_msgs::Point &point, const std::string &frame_id, double yaw)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = frame_id;

    pose_stamped.pose.position = point;
    tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);
    quaternionTFToMsg(quat, pose_stamped.pose.orientation);

    return pose_stamped;
}

void StateMachine::addGoalPoint(double x, double y, double z)
{
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    goalpoints.push_back(point);
}

void StateMachine::state_machine_mission(const ros::TimerEvent &t)
{
    if (state_ == State::takeoff)
    {
        takeoff();
    }
    else if (state_ == State::to_cave)
    {
        to_cave();
    }
    else if (state_ == State::hover)
    {
        hover();
    }
    else if (state_ == State::landing)
    {
        landing();
    }
    else if (state_ == State::turn)
    {
        turn();
    }
    else if (state_ == State::forward)
    {
        forward();
    }
}

void StateMachine::takeoff()
{
    ROS_INFO_ONCE("Drone is taking off!");
    if (!goal_sent_once)
    {
        goal_sent_once = 1;
        goalpoint = StateMachine::getNextGoalPoint();
        ros::Duration(1).sleep();
        auto goal_pose_stamped = StateMachine::pointToPoseStamped(goalpoint, "world");

        goal_position_pub_.publish(goal_pose_stamped);
        ROS_INFO("Published goal position: [%f, %f, %f]", goalpoint.x, goalpoint.y, goalpoint.z);
    }
    // ros::Duration(1).sleep();

    if (goal_reached())
    {
        goal_sent_once = 0;
        state_ = State::to_cave;
    }
}

void StateMachine::to_cave()
{

    ROS_INFO_ONCE("Drone is flying to cave!");
    if (!goal_sent_once)
    {
        goal_sent_once = 1;
        goalpoint = StateMachine::getNextGoalPoint();
        // ros::Duration(1).sleep();
        auto goal_pose_stamped = StateMachine::pointToPoseStamped(goalpoint, "world", -1.5708 * 2);

        goal_position_pub_.publish(goal_pose_stamped);
        ROS_INFO("Published goal position: [%f, %f, %f]", goalpoint.x, goalpoint.y, goalpoint.z);
    }

    if (goal_reached())
    {
        goal_sent_once = 0;
        // goalpoint = StateMachine::getNextGoalPoint();
        if (current_goal_index == 3)
        {
            state_ = State::hover;
            yaw_des = -1.5708; //-90 deg
            set_position();
        }
    }
}

double calculateYawForWaypoint(const geometry_msgs::Point &current, const geometry_msgs::Point &next)
{
    double delta_x = next.x - current.x;
    double delta_y = next.y - current.y;
    return std::atan2(delta_y, delta_x);
}

void StateMachine::hover()
{
    ROS_INFO_ONCE("Drone is hovering!");
}

void StateMachine::landing()
{
    ROS_INFO_ONCE("Drone is landing!");
    tf::Vector3 pos(cur_position[0], cur_position[1], 0);
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    set_waypoint(pos, q);
}

void StateMachine::turn()
{
    ROS_INFO_ONCE("Drone is turning!");
    tf::Vector3 pos(cur_position[0], cur_position[1], cur_position[2]);

    tf::Quaternion q;
    q.setRPY(0, 0, yaw_des);

    set_waypoint(pos, q);

    if (in_range(yaw_des - 0.01, yaw_des + 0.01, yaw_))
    {
        state_ = State::forward;
        set_position();
    }
}

void StateMachine::forward()
{
    ROS_INFO_ONCE("Drone is flying forward!");
    tf::Vector3 pos(cur_position[0], cur_position[1] - 3, cur_position[2]);
    tf::Quaternion q;
    q.setRPY(0, 0, yaw_des);
    set_waypoint(pos, q);

    if (in_range(pos_[1] - tol_, pos_[1] + tol_, cur_position[1] - 5))
    {
        state_ = State::hover;
        set_position();
    }
}

void StateMachine::onCurrentState(
    const nav_msgs::Odometry &cur_state)
{

    pos_ << cur_state.pose.pose.position.x,
        cur_state.pose.pose.position.y,
        cur_state.pose.pose.position.z;
    vel_ << cur_state.twist.twist.linear.x,
        cur_state.twist.twist.linear.y,
        cur_state.twist.twist.linear.z;
    omega_ << cur_state.twist.twist.angular.x,
        cur_state.twist.twist.angular.y,
        cur_state.twist.twist.angular.z;

    tf::quaternionMsgToTF(cur_state.pose.pose.orientation, quat_);
    yaw_ = tf::getYaw(quat_);
}

bool StateMachine::in_range(double low, double high, double x)
{
    return ((x - high) * (x - low) <= 0);
}

bool StateMachine::goal_reached()
{
    return (in_range(goalpoint.x - tol_, goalpoint.x + tol_, pos_[0]) &&
            in_range(goalpoint.y - tol_, goalpoint.y + tol_, pos_[1]) &&
            in_range(goalpoint.z - tol_, goalpoint.z + tol_, pos_[2]));
}

void StateMachine::set_waypoint(tf::Vector3 pos, tf::Quaternion q,
                                tf::Vector3 lin_vel, tf::Vector3 ang_vel, tf::Vector3 lin_acc)
{
    tf::Transform desired_pos(tf::Transform::getIdentity());
    geometry_msgs::Twist vel;
    geometry_msgs::Twist acc;

    desired_pos.setOrigin(pos);
    desired_pos.setRotation(q);

    vel.linear.x = lin_vel.getX();
    vel.linear.y = lin_vel.getY();
    vel.linear.z = lin_vel.getZ();

    vel.angular.x = ang_vel.getX();
    vel.angular.y = ang_vel.getY();
    vel.angular.z = ang_vel.getZ();

    acc.linear.x = lin_acc.getX();
    acc.linear.y = lin_acc.getY();
    acc.linear.z = lin_acc.getZ();

    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    msg.transforms.resize(1);
    msg.transforms[0].translation.x = desired_pos.getOrigin().x();
    msg.transforms[0].translation.y = desired_pos.getOrigin().y();
    msg.transforms[0].translation.z = desired_pos.getOrigin().z();
    msg.transforms[0].rotation.x = desired_pos.getRotation().getX();
    msg.transforms[0].rotation.y = desired_pos.getRotation().getY();
    msg.transforms[0].rotation.z = desired_pos.getRotation().getZ();
    msg.transforms[0].rotation.w = desired_pos.getRotation().getW();

    msg.velocities.resize(1);
    msg.velocities[0] = vel;
    msg.accelerations.resize(1);
    msg.accelerations[0] = acc;

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.points.push_back(msg);
    desired_state_pub_.publish(trajectory_msg);

    br.sendTransform(tf::StampedTransform(desired_pos, ros::Time::now(),
                                          "world", "av-desired"));
}

void StateMachine::set_position()
{
    cur_position << pos_[0], pos_[1], pos_[2];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_machine");
    ROS_INFO_ONCE("State machine initialized.");
    StateMachine statemachine1;
    ros::spin();
}