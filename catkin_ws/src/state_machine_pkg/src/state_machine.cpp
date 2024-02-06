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

#include <eigen3/Eigen/Dense>

#include <cstdlib>
#include <iostream>


StateMachine::StateMachine(): waypoint_navigation_launched(false) {
    double x, y, z;
    x= -38.0;
    y= 10.0;
    z=6.9;

    addGoalPoint(-38.0, 10.0, 10.0); // take off from initial position
    addGoalPoint(-55, 0.84, 15.0); // first lamp position at the outside
    addGoalPoint(-321, 10.0, 15.0); // cave entrance
    addGoalPoint(-500, 0.0, 10.0);
    addGoalPoint(-599, -9.0, 10.0);
    addGoalPoint(-599, -5, 10.0);
    addGoalPoint(-599, -2, 3.0);

    
    origin_ = tf::Vector3(x, y, z);
    
    desired_state_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);
    current_state_sub_ = nh.subscribe("current_state_est", 1, &StateMachine::onCurrentState, this);
    goal_position_pub_ = nh.advertise<geometry_msgs::Point>("goal_position", 1);

                                  
    state_machine_timer_ = nh.createTimer(ros::Duration(sim_interval_),
    &StateMachine::state_machine_mission, this);
  }

geometry_msgs::Point StateMachine::getNextGoalPoint() {
    if (current_goal_index < goalpoints.size()) {
        return goalpoints[current_goal_index++];
    } else {
        // Return the last point or handle the end of the vector as needed
        return goalpoints.back();
    }
}


void StateMachine::addGoalPoint(double x, double y, double z) {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    goalpoints.push_back(point);
}


void StateMachine::state_machine_mission(const ros::TimerEvent& t) {
    if (state_ == State::takeoff) { takeoff(); }
    else if (state_ == State::to_cave) { to_cave(); }
    else if (state_ == State::hover) { hover(); }
    else if (state_ == State::explore) { explore();}
    else if (state_ == State::landing) { landing();}
    else if (state_ == State::turn) { turn();}
}


void StateMachine::takeoff() {  
    ROS_INFO_ONCE("Drone is taking off!");
    if(!goal_sent_once){
          goal_sent_once=1;
          goalpoint = StateMachine::getNextGoalPoint();
          ros::Duration(1).sleep();
          goal_position_pub_.publish(goalpoint);
          ROS_INFO("Published goal position: [%f, %f, %f]", goalpoint.x, goalpoint.y, goalpoint.z); }
          // ros::Duration(1).sleep();

    if(goal_reached()) {
        goal_sent_once=0;
        state_ = State::to_cave;
    } 
}


void StateMachine::to_cave() {
    
    ROS_INFO_ONCE("Drone is flying to cave!");
    if(!goal_sent_once){
      goal_sent_once=1;
      goalpoint = StateMachine::getNextGoalPoint();
      // ros::Duration(1).sleep();
      goal_position_pub_.publish(goalpoint);
      ROS_INFO("Published goal position: [%f, %f, %f]", goalpoint.x, goalpoint.y, goalpoint.z); }
      

    if(goal_reached()) {
        goal_sent_once=0;
        // goalpoint = StateMachine::getNextGoalPoint();
        if(current_goal_index == 7){
          state_ = State::turn;
        }

    }
}



void StateMachine::hover() {
    ROS_INFO_ONCE("Drone is hovering!");
}


void StateMachine::explore() {
  ROS_INFO_ONCE("Drone is exploring!");
  // TODO
}  

void StateMachine::landing() {
    ROS_INFO_ONCE("Drone is landing!");
    // TODO
}


void StateMachine::turn() {
    ROS_INFO_ONCE("Drone is turning!");
    tf::Vector3 pos(pos_[0], pos_[1], pos_[2]);
    
    double angle_radians = -90 * M_PI / 180.0;
    tf::Quaternion q;
    q.setRPY(0, 0, angle_radians);

    set_waypoint(pos, q);
}




void StateMachine::onCurrentState(
    const nav_msgs::Odometry& cur_state) {

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


bool StateMachine::in_range(double low, double high, double x) { 
  return ((x-high)*(x-low) <= 0);
}

bool StateMachine::goal_reached() { 
  return ( in_range(goalpoint.x - tol_, goalpoint.x + tol_, pos_[0]) &&
          in_range(goalpoint.y - tol_, goalpoint.y + tol_, pos_[1]) &&
          in_range(goalpoint.z - tol_, goalpoint.z + tol_, pos_[2]) );
}

void StateMachine::set_waypoint(tf::Vector3 pos, tf::Quaternion q,
 tf::Vector3 lin_vel, tf::Vector3 ang_vel, tf::Vector3 lin_acc) {
  tf::Transform desired_pos(tf::Transform::getIdentity());
  geometry_msgs::Twist vel;
  geometry_msgs::Twist acc;

  // WAYPOINT INITIALIZATION. 
  // Setting desired position and orientation
  desired_pos.setOrigin(pos);  //  Setting desired point
  desired_pos.setRotation(q);

   // Defining angular and linear velocity
  vel.linear.x = lin_vel.getX();
  vel.linear.y = lin_vel.getY();
  vel.linear.z = lin_vel.getZ();

  vel.angular.x = ang_vel.getX();
  vel.angular.y = ang_vel.getY();
  vel.angular.z = ang_vel.getZ();

  // Defining linear acceleration
  acc.linear.x = lin_acc.getX();
  acc.linear.y = lin_acc.getY();
  acc.linear.z = lin_acc.getZ();

  //  Making a message to send to rotors
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
                                              "world", "av-desired"));}



int main(int argc, char **argv) {
    ros::init(argc, argv, "state_machine");
    ROS_INFO_ONCE("State machine initialized.");
    StateMachine statemachine1;
    ros::spin();
}
