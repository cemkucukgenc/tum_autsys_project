/*
 *   OctomapPlanner
 *
 *   Copyright (C) 2018  ArduPilot
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *   Author Ayush Gaud <ayush.gaud[at]gmail.com>
 */

#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/config.h>
#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/octree/octree.h>
#include <mutex>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner
{
public:
	Planner();
	~Planner();

	// void setStart(double x, double y, double z);

	void setGoal(const geometry_msgs::PointConstPtr &msg);

	void plan(void);

	// bool replan(void);

	void updateMap(const octomap_msgs::OctomapConstPtr &msg);

	void updateOdom(const nav_msgs::Odometry &msg);

	nav_msgs::Path pathMsg(og::PathGeometric *pathGeo);

	nav_msgs::Path vectorMsg(const std::vector<Eigen::Vector3d> &pathVector);

	std::vector<std::tuple<double, double, double>> getSmoothPath();

	bool isStateValid(const ob::State *state);

	// ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr &si);

private:
	ros::NodeHandle nh;

	ros::Subscriber current_state_sub, octomap_sub, frontier_goal_sub;
	ros::Publisher planned_path_pub;

	// construct the state space we are planning in
	ob::StateSpacePtr space;

	ob::ScopedState<ob::RealVectorStateSpace> start;

	ob::ScopedState<ob::RealVectorStateSpace> goal;

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si;

	// create a problem instance
	ob::ProblemDefinitionPtr pdef;

	// Planner instance
	ob::PlannerPtr o_plan;

	ob::RealVectorBounds bounds = ob::RealVectorBounds(3);

	og::PathGeometric *path_smooth = NULL;

	// bool replan_flag = true;

	std::mutex mutex_flag;

	bool plan_flag;

	std::shared_ptr<fcl::CollisionObject<double>> treeObj;

	std::shared_ptr<fcl::CollisionObject<double>> aircraftObject;
};

#endif