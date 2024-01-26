#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>

#define PI M_PI
const tf::Vector3 zero_vec(0, 0, 0);

enum class State {
takeoff,
to_cave,
hover,
explore,
landing
};

class StateMachine {
 private:
  ros::NodeHandle nh;

  ros::Subscriber cmd_vel_sub_, current_state_sub_;
  ros::Publisher desired_state_pub_;
  tf::TransformBroadcaster br;
  geometry_msgs::Twist cmd_twist_;

  State state_ = State::takeoff;
  ros::Time start_ = ros::Time::now();
  ros::Timer state_machine_timer_;

  tf::Vector3 origin_;
  Eigen::Vector3d pos_, vel_, omega_;
  tf::Quaternion quat_;
  double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
  Eigen::Vector3d hover_pos_;
  
  const float takeoff_height_ = 10.0;
  const float sim_interval_ = 0.1;
  const double tol_ = 0.5;

 public:
  StateMachine();

  void onCurrentState(const nav_msgs::Odometry& current_state);
  void onCmdVel(geometry_msgs::Twist cmd_vel);
  void set_waypoint(tf::Vector3 pos, tf::Quaternion q, tf::Vector3 lin_vel = zero_vec, tf::Vector3 ang_vel = zero_vec, tf::Vector3 lin_acc = zero_vec);
  void set_hovering_height();
  void rotate_z(tf::Vector3& vec, const double yaw);

  void state_machine_mission(const ros::TimerEvent& t);
  void takeoff();
  void to_cave();
  void hover();
  void explore();
  void landing();

  bool in_range(double low, double high, double x);
  tf::Vector3 twist_to_pos(geometry_msgs::Twist twist);
  tf::Quaternion twist_to_quat(geometry_msgs::Twist twist);


};


#endif  //  STATE_MACHINE_H
