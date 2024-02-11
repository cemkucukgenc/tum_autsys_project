#include <fstream>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <aeplanner_evaluation/Coverage.h>
#include <std_srvs/Empty.h>

#include <actionlib/client/simple_action_client.h>
#include <aeplanner/Node.h>
#include <aeplanner/aeplannerAction.h>
#include <rpl_exploration/FlyToAction.h>
#include <rrtplanner/rrtAction.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

bool current_goal_reached = true;

void pointReachedCallback(std_msgs::Bool msg)
{
  if (msg.data)
  {
    ROS_INFO("Current goal point is reached!");
    current_goal_reached = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ROS_INFO("Started exploration");

  // Open logfile;
  std::string path = ros::package::getPath("rpl_exploration");
  std::ofstream logfile, pathfile, timefile;
  logfile.open(path + "/data/logfile.csv");
  pathfile.open(path + "/data/path.csv");
  timefile.open(path + "/data/time.csv");
  
  ros::Publisher goals_pub(nh.advertise<geometry_msgs::PoseArray>("/red/nbvp/goals", 1));
  ros::Publisher pub(nh.advertise<geometry_msgs::PoseStamped>("/red/nbvp/target", 10));
  ros::Publisher comp_time_pub(nh.advertise<std_msgs::Float64>("/red/comp_time", 10));

  ros::Subscriber point_reached_sub = nh.subscribe("/red/nbvp/point_reached", 1, &pointReachedCallback);

  ros::ServiceClient coverage_srv =
      nh.serviceClient<aeplanner_evaluation::Coverage>("/aeplanner/get_coverage");

  // wait for fly_to server to start
  // ROS_INFO("Waiting for fly_to action server");
  actionlib::SimpleActionClient<rpl_exploration::FlyToAction> ac("fly_to", true);
  // ac.waitForServer(); //will wait for infinite time
  // ROS_INFO("Fly to ction server started!");

  // wait for aep server to start
  ROS_INFO("Waiting for aeplanner action server");
  actionlib::SimpleActionClient<aeplanner::aeplannerAction> aep_ac("make_plan",
                                                                   true);
  aep_ac.waitForServer();  // will wait for infinite time
  ROS_INFO("aeplanner action server started!");

  // wait for fly_to server to start
  ROS_INFO("Waiting for rrt action server");
  actionlib::SimpleActionClient<rrtplanner::rrtAction> rrt_ac("rrt", true);
  // rrt_ac.waitForServer(); //will wait for infinite time
  ROS_INFO("rrt Action server started!");

  // Get current pose
  geometry_msgs::PoseStamped::ConstPtr init_pose =
      ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pose_est");
  double init_yaw = tf2::getYaw(init_pose->pose.orientation);
  // Up 2 meters and then forward one meter
  double initial_positions[8][4] = {
    { init_pose->pose.position.x, init_pose->pose.position.y,
      init_pose->pose.position.z + 0, init_yaw },
    { init_pose->pose.position.x + 1.0 * std::cos(init_yaw),
      init_pose->pose.position.y + 1.0 * std::sin(init_yaw),
      init_pose->pose.position.z + 0, init_yaw },
  };

  // This is the initialization motion, necessary that the known free space
  // allows the planning of initial paths.
  ROS_INFO("Starting the planner: Performing initialization motion");
  geometry_msgs::Pose last_pose;
  geometry_msgs::PoseArray goals;

  for (int i = 0; i < 2; ++i)
  {
    rpl_exploration::FlyToGoal goal;
    goal.pose.pose.position.x = initial_positions[i][0];
    goal.pose.pose.position.y = initial_positions[i][1];
    goal.pose.pose.position.z = initial_positions[i][2];
    goal.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(initial_positions[i][3]);
    last_pose = goal.pose.pose;

    ac.sendGoal(goal);
    
    goals.header.stamp = ros::Time::now();
    goals.header.frame_id = "world";
    goals.poses.push_back(goal.pose.pose);
    // ac.waitForResult(ros::Duration(0));
  }
  ROS_INFO_STREAM("Sending initial goal...");
  goals_pub.publish(goals);
  while (!current_goal_reached)
    {
      ros::spinOnce();
      ROS_WARN("waiting flag reached - initial motion.");
      ros::Duration(0.1).sleep();
    }
    current_goal_reached = false;

  // Start planning: The planner is called and the computed path sent to the
  // controller.
  int iteration = 0;
  int actions_taken = 1;

  ros::Time start = ros::Time::now();
  while (ros::ok())
  {
    ros::Time computation_start_time = ros::Time::now();
    ROS_INFO_STREAM("Planning iteration " << iteration);
    aeplanner::aeplannerGoal aep_goal;
    aep_goal.header.stamp = ros::Time::now();
    aep_goal.header.seq = iteration;
    aep_goal.header.frame_id = "world";
    aep_goal.actions_taken = actions_taken;
    aep_ac.sendGoal(aep_goal);

    // Publishing last pose - not neccessary in our setup
    while (!aep_ac.waitForResult(ros::Duration(0.05)))
    {
      // pub.publish(last_pose);
      geometry_msgs::PoseArray goals;
      goals.header.stamp = ros::Time::now();
      goals.header.frame_id = "world";
      goals.poses.push_back(last_pose);
      // goals_pub.publish(goals);
    }

    ros::Duration fly_time;
    if (aep_ac.getResult()->is_clear)
    {
      actions_taken = 0;
      ros::Time s = ros::Time::now();
      geometry_msgs::PoseStamped goal_pose = aep_ac.getResult()->pose;
      // Write path to file
      pathfile << goal_pose.pose.position.x << ", " << goal_pose.pose.position.y
              << ", " << goal_pose.pose.position.z << ", n" << std::endl;
      std::cout << goal_pose.pose.position.x << ", " << goal_pose.pose.position.y
              << ", " << goal_pose.pose.position.z << std::endl;
      last_pose = goal_pose.pose;
      rpl_exploration::FlyToGoal goal;
      goal.pose = goal_pose;
      std_msgs::Float64 comp_time;
      comp_time.data = (ros::Time::now() - computation_start_time).toSec();
      ROS_WARN_STREAM ("COMP TIME: " << comp_time);
      comp_time_pub.publish(comp_time);
      timefile << comp_time.data << std::endl;
      pub.publish(goal_pose);
      while (!current_goal_reached)
      {
        ros::spinOnce();
        // ROS_WARN("waiting flag reached.");
        ros::Duration(0.1).sleep();
      }
      current_goal_reached = false;
      ac.sendGoal(goal);
      // ROS_WARN("Izlazim iz if! ");
      // ac.waitForResult(ros::Duration(0));        

      fly_time = ros::Time::now() - s;
    }
    else
    {
      std::cout << "u else" << std::endl;
      rrtplanner::rrtGoal rrt_goal;
      rrt_goal.start.header.stamp = ros::Time::now();
      rrt_goal.start.header.frame_id = "world";
      rrt_goal.start.pose = last_pose;
      if (!aep_ac.getResult()->frontiers.poses.size())
      {
        ROS_WARN("Exploration complete!");
        break;
      }
      for (auto it = aep_ac.getResult()->frontiers.poses.begin();
          it != aep_ac.getResult()->frontiers.poses.end(); ++it)
      {
        rrt_goal.goal_poses.poses.push_back(*it);
      }

      rrt_ac.sendGoal(rrt_goal);
      while (!rrt_ac.waitForResult(ros::Duration(0.05)))
      {
        ROS_WARN("While loop rrt_ac.waitForResult!");
        // pub.publish(last_pose);
        geometry_msgs::PoseArray goals;
        goals.header.stamp = ros::Time::now();
        goals.header.frame_id = "world";
        goals.poses.push_back(last_pose);
        // goals_pub.publish(goals);
      }

      nav_msgs::Path path = rrt_ac.getResult()->path;
      std::cout << "Path size: " << path.poses.size() << std::endl;
      ROS_ERROR_STREAM("Path size: " << path.poses.size());

      ros::Time s = ros::Time::now();
      geometry_msgs::PoseArray goals;
      rpl_exploration::FlyToGoal goal;
      if (path.poses.size() != 0)
      {
        for (int i = path.poses.size() - 1; i >= 0; --i)
        {
          geometry_msgs::PoseStamped new_goal_pose = path.poses[i];
          // Write path to file
          pathfile << new_goal_pose.pose.position.x << ", " << new_goal_pose.pose.position.y << ", "
                  << new_goal_pose.pose.position.z << ", f" << std::endl;
          std::cout << new_goal_pose.pose.position.x << ", " << new_goal_pose.pose.position.y << ", "
                  << new_goal_pose.pose.position.z << ", f" << std::endl;

          last_pose = new_goal_pose.pose;
          goal.pose = new_goal_pose;
          // while (!current_goal_reached)
          // {
          //   ros::spinOnce();
          //   ROS_WARN("Waiting flag reached.");
          //   ros::Duration(0.1).sleep();
          // }
          // current_goal_reached = false;
          // pub.publish(new_goal_pose);
          
          goals.header.stamp = ros::Time::now();
          goals.header.frame_id = "world";
          goals.poses.push_back(goal.pose.pose);

        }
        goals_pub.publish(goals);

        while (!current_goal_reached)
        {
          ros::spinOnce();
          // ROS_WARN("Waiting flag reached.");
          ros::Duration(0.1).sleep();
        }
        current_goal_reached = false;
        std::cout << "gol koji saljem: " << goal.pose.pose.position.x << goal.pose.pose.position.y << std::endl;
        ac.sendGoal(goal);
        // ROS_WARN("Waiting sc.waitForResult.");
        // ac.waitForResult(ros::Duration(0));
        actions_taken = -1;
        fly_time = ros::Time::now() - s;
      }
    }
    ros::Duration elapsed = ros::Time::now() - start;
    aeplanner_evaluation::Coverage srv;
    coverage_srv.call(srv);
 
    ROS_INFO_STREAM("Iteration: "       << iteration << "  " <<
                    "Time: "            << elapsed << "  " <<
                    "SPC: "             << aep_ac.getResult()->sampling_time.data + 
                                          aep_ac.getResult()->planning_time.data + 
                                          aep_ac.getResult()->collision_check_time.data << "  " << 
                    "Sampling: "        << aep_ac.getResult()->sampling_time.data << "  " <<
                    "Planning: "        << aep_ac.getResult()->planning_time.data << "  " <<
                    "Flying: "          << fly_time << " " <<
                    "Elapsed: "         << elapsed << " " <<
                    "Explored: "        << srv.response.coverage);

    logfile << iteration << ", " 
            << elapsed << ", "
            << aep_ac.getResult()->sampling_time.data + 
              aep_ac.getResult()->planning_time.data + 
              aep_ac.getResult()->collision_check_time.data << ", "
            << aep_ac.getResult()->sampling_time.data << ", "
            << aep_ac.getResult()->planning_time.data << ", "
            << aep_ac.getResult()->collision_check_time.data << ", "
            << fly_time << ", "
            << aep_ac.getResult()->tree_size << ", "
            << srv.response.coverage << std::endl;

    iteration++;
  }

  pathfile.close();
  logfile.close();
}
