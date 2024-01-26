#include <state_machine.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Dense>

StateMachine::StateMachine() {
    double x, y, z;
    x= -38.0;
    y= 10.0;
    z=6.9;
    
    origin_ = tf::Vector3(x, y, z);
    
    desired_state_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
    current_state_sub_ = nh.subscribe("current_state_est", 1, &StateMachine::onCurrentState, this);
    cmd_vel_sub_ = nh.subscribe("cmd_vel", 1000, &StateMachine::onCmdVel, this);
                                  
    state_machine_timer_ = nh.createTimer(ros::Duration(sim_interval_),
    &StateMachine::state_machine_mission, this);
  }

void StateMachine::state_machine_mission(const ros::TimerEvent& t) {
    if (state_ == State::takeoff) { takeoff(); }
    else if (state_ == State::to_cave) { to_cave(); }
    else if (state_ == State::hover) { hover(); }
    else if (state_ == State::explore) { explore();}
    else if (state_ == State::landing) { landing();}
}

void StateMachine::takeoff() {
  ROS_INFO_ONCE("Drone is taking off!");
  tf::Vector3 pos(origin_.getX(), origin_.getY(), takeoff_height_);
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  set_waypoint(pos, q);

  // if (in_range(takeoff_height_-tol_, takeoff_height_+tol_, pos_[2])) { 
  //   state_ = State::hover;
  //   set_hovering_height();
  // }

    if (in_range(takeoff_height_-tol_, takeoff_height_+tol_, pos_[2])) { 
    state_ = State::to_cave;
  }
}

void StateMachine::to_cave() {
   ROS_INFO_ONCE("Drone is flying to cave!");
   // TODO
}



void StateMachine::hover() {
    ROS_INFO_ONCE("Drone is hovering!");
    tf::Vector3 pos(hover_pos_[0], hover_pos_[1], hover_pos_[2]);
    tf::Quaternion q;
    q.setRPY(0, 0, 0);

    set_waypoint(pos, q);

    // if (ros::Duration(10.0).sleep()) {
    //   state_ = State::explore;
    // }
}


void StateMachine::explore() {
  ROS_INFO_ONCE("Drone is exploring!");
  // TODO
}  

void StateMachine::landing() {
    ROS_INFO_ONCE("Drone is landing!");
    tf::Vector3 pos(hover_pos_[0], hover_pos_[1], 0); 
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    set_waypoint(pos, q);

    if (in_range(-tol_, tol_, pos_[2])) { 
      state_ = State::hover;
      set_hovering_height();
    }
}



tf::Vector3 StateMachine::twist_to_pos(geometry_msgs::Twist twist) {
  tf::Vector3 current_pos = tf::Vector3(pos_[0], pos_[1], takeoff_height_);
  tf::Vector3 delta_pos = tf::Vector3(sim_interval_ * twist.linear.x, 
                         sim_interval_ * twist.linear.y, 0);

  tf::Transform body_to_world_tf(tf::Transform::getIdentity());
  body_to_world_tf.setRotation(quat_);

  delta_pos = body_to_world_tf * delta_pos;

  return current_pos + delta_pos;
}

tf::Quaternion StateMachine::
twist_to_quat(geometry_msgs::Twist twist) {
  tf::Quaternion rot;
  // Set desired rotation
  // Only Yaw is important for the controller
  rot.setRPY(0, 0, yaw_ + (twist.angular.z * sim_interval_ ));
  return rot.normalize();
}

void StateMachine::onCmdVel(geometry_msgs::Twist cmd_vel) {
  cmd_twist_.angular.x = cmd_vel.angular.x;
  cmd_twist_.angular.y = cmd_vel.angular.y;
  cmd_twist_.angular.z = cmd_vel.angular.z;

  cmd_twist_.linear.x = cmd_vel.linear.x;
  cmd_twist_.linear.y = cmd_vel.linear.y;
  cmd_twist_.linear.z = cmd_vel.linear.z;
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

  desired_state_pub_.publish(msg);

  br.sendTransform(tf::StampedTransform(desired_pos, ros::Time::now(),
                                              "world", "av-desired"));

}

bool StateMachine::in_range(double low, double high, double x) { 
  return ((x-high)*(x-low) <= 0);
}

void StateMachine::set_hovering_height() {
  hover_pos_ << pos_[0], pos_[1], pos_[2];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_machine");
    ROS_INFO_ONCE("State machine initialized.");
    StateMachine statemachine1;
    ros::spin();
}
