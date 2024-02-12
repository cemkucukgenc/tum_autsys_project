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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#define PI M_PI
const tf::Vector3 zero_vec(0, 0, 0);

enum class State {
takeoff,
to_cave,
hover,
explore,
landing,
turn,
forward
};

class StateMachine {
 private:
 
  bool waypoint_navigation_launched;
  ros::NodeHandle nh;

  ros::Subscriber cmd_vel_sub_, current_state_sub_;
  ros::Publisher desired_state_pub_;
  ros::Publisher goal_position_pub_;
      ros::Subscriber best_frontier_marker_sub_;
    ros::Subscriber exploration_goal_sub_;
  tf::TransformBroadcaster br;
  geometry_msgs::Twist cmd_twist_;

  State state_ = State::takeoff;
  ros::Time start_ = ros::Time::now();
  ros::Timer state_machine_timer_;

  tf::Vector3 origin_;
  Eigen::Vector3d pos_, vel_, omega_;
  tf::Quaternion quat_;
  double yaw_ = 0.0;
  
  const float takeoff_height_ = 10.0;
  const float sim_interval_ = 0.1;
  const double tol_ = 0.5;
  
  geometry_msgs::Point goalpoint;
  std::vector<geometry_msgs::Point> goalpoints;
  size_t current_goal_index = 0;
  bool goal_sent_once = 0;
  
  Eigen::Vector3d cur_position;
  double yaw_des = 0;


 public:
  StateMachine();

  void onCurrentState(const nav_msgs::Odometry& current_state);

  void state_machine_mission(const ros::TimerEvent& t);
  void takeoff();
  void to_cave();
  void hover();
  void explore();
  void landing();
  void turn();
  void forward();
      void markerCallback(const visualization_msgs::Marker::ConstPtr& msg);
    void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  
  geometry_msgs::Point getNextGoalPoint();
  void addGoalPoint(double x, double y, double z);

  bool in_range(double low, double high, double x);
  bool goal_reached();
  
  void set_waypoint(tf::Vector3 pos, tf::Quaternion q,
   tf::Vector3 lin_vel = zero_vec, tf::Vector3 ang_vel = zero_vec,
    tf::Vector3 lin_acc = zero_vec);
  
  void set_position();
  void set_yaw();

};


#endif  //  STATE_MACHINE_H
