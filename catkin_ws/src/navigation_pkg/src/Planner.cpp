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

#include "Planner.h"

// Constructor
Planner::Planner(void) : space(ob::StateSpacePtr(new ob::RealVectorStateSpace(3))),
						 goal(ob::ScopedState<ob::RealVectorStateSpace>(space)),
						 start(ob::ScopedState<ob::RealVectorStateSpace>(space)),
						 mutex_flag(),
						 plan_flag(false),
						 bounds(3)
{
	current_state_sub = nh.subscribe("/current_state_est", 1, &Planner::updateOdom, this);
	octomap_sub = nh.subscribe("octomap_full", 1, &Planner::updateMap, this);
	frontier_goal_sub = nh.subscribe("frontier_goal", 1, &Planner::setGoal, this);
	planned_path_pub = nh.advertise<nav_msgs::Path>("planned_path", 0);

	// Change in Box size deeply affects the collision checking
	// It is quite possible that planner will result in different paths for different box sizes
	aircraftObject = std::make_shared<fcl::CollisionObject<double>>(std::make_shared<fcl::Box<double>>(3, 3, 3));
	// fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.15)));
	// tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
	// treeObj = std::make_shared<fcl::CollisionObject<double>>((std::shared_ptr<fcl::CollisionGeometry<double>>(tree)));

	// space = ob::StateSpacePtr(new ob::RealVectorStateSpace(3));

	// // create a start state
	// ob::ScopedState<ob::RealVectorStateSpace> start(space);

	// // create a goal state
	// ob::ScopedState<ob::RealVectorStateSpace> goal(space);

	// // set the bounds for the R^3
	// ob::RealVectorBounds bounds(3);

	bounds.setLow(0, -10);
	bounds.setHigh(0, 10);
	bounds.setLow(1, -10);
	bounds.setHigh(1, 10);
	bounds.setLow(2, 0.5);
	bounds.setHigh(2, 3.5);

	space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	// construct an instance of  space information from this state space
	si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

	// // start->setXYZ(0,0,0);
	// start->values[0] = 0;
	// start->values[1] = 0;
	// start->values[2] = 0;
	// // start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// // start.random();

	// // goal->setXYZ(0,0,0);
	// goal->values[0] = 0;
	// goal->values[1] = 0;
	// goal->values[2] = 0;

	// // goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// // goal.random();

	// set state validity checking for this space
	si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1));

	// create a problem instance
	pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

	// // set the start and goal states
	// pdef->setStartAndGoalStates(start, goal);

	// // set Optimizattion objective
	// pdef->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(si));

	// // create a planner for the defined space
	// o_plan = ob::PlannerPtr(new og::InformedRRTstar(si));

	// // set the problem we are trying to solve for the planner
	// o_plan->setProblemDefinition(pdef);

	// // perform setup steps for the planner
	// o_plan->setup();

	ROS_INFO("Planner Initialized");
}

// Destructor
Planner::~Planner()
{
}

// void Planner::setStart(double x, double y, double z)
// {
// 	ob::ScopedState<ob::RealVectorStateSpace> start(space);
// 	start->values[0] = x;
// 	start->values[1] = y;
// 	start->values[2] = z;
// 	ob::State *state = space->allocState();
// 	state->as<ob::RealVectorStateSpace::StateType>()->values = start->values;
// 	if (isStateValid(state)) // Check if the start state is valid
// 	{
// 		pdef->clearStartStates();
// 		pdef->addStartState(start);
// 		ROS_INFO_STREAM("Start point set to: " << x << " " << y << " " << z);
// 		return true;
// 	}
// 	else
// 	{
// 		ROS_ERROR("Start state: " << x << " " << y << " " << z << " invalid");
// 		return false;
// 	}
// }

void Planner::setGoal(const geometry_msgs::PointConstPtr &msg)
{
	std::lock_guard<std::mutex> lock(mutex_flag);

	// Create a state representing the goal
	// ob::ScopedState<ob::RealVectorStateSpace> goal(space);
	goal[0] = msg->x;
	goal[1] = msg->y;
	goal[2] = msg->z;
	// pdef->clearGoal();
	plan_flag = true;
	pdef->clearSolutionPaths();
	// pdef->setGoalState(goal);
	ob::State *state = space->allocState();
	state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
	state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
	state->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];
	// state->as<ob::RealVectorStateSpace::StateType>()->values = goal->values;

	if (isStateValid(state)) // Check if the goal state is valid
	{
		pdef->setStartAndGoalStates(start, goal);
		pdef->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(si));
		// o_plan = ob::PlannerPtr(new og::InformedRRTstar(si));
		o_plan = ob::PlannerPtr(new og::RRTstar(si));
		o_plan->setProblemDefinition(pdef);
		o_plan->setup();
		plan();
		plan_flag = false;
		ROS_INFO_STREAM("Goal point set to: " << msg->x << " " << msg->y << " " << msg->z);
	}
	else
	{
		ROS_ERROR_STREAM("Goal state: " << msg->x << " " << msg->y << " " << msg->z << " invalid");
	}

	space->freeState(state);
}

void Planner::updateMap(const octomap_msgs::OctomapConstPtr &msg)
{
	std::lock_guard<std::mutex> lock(mutex_flag);
	if (plan_flag)
		return;

	// Convert the received Octomap message to an Octree
	auto octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));

	// Retrieve the bounding box of the Octree
	double minBounds[3], maxBounds[3];
	octree->getMetricMin(minBounds[0], minBounds[1], minBounds[2]);
	octree->getMetricMax(maxBounds[0], maxBounds[1], maxBounds[2]);

	// convert octree to collision object
	fcl::OcTree<double> *fclTree = new fcl::OcTree<double>(octree);
	auto collisionGeometry = std::shared_ptr<fcl::CollisionGeometry<double>>(fclTree);
	treeObj = std::make_shared<fcl::CollisionObject<double>>(collisionGeometry);

	// Set the bounds of the planning space based on the Octree bounding box
	bounds.setLow(0, minBounds[0]);
	bounds.setHigh(0, maxBounds[0]);
	bounds.setLow(1, minBounds[1]);
	bounds.setHigh(1, maxBounds[1]);
	bounds.setLow(2, minBounds[2]);
	bounds.setHigh(2, maxBounds[2]);

	space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
}

void Planner::updateOdom(const nav_msgs::Odometry &msg)
{
	start[0] = msg.pose.pose.position.x;
	start[1] = msg.pose.pose.position.y;
	start[2] = msg.pose.pose.position.z;
}

nav_msgs::Path Planner::pathMsg(og::PathGeometric *pathGeo)
{
	std::vector<Eigen::Vector3d> pathPoints;
	const auto &states = pathGeo->getStates();

	for (size_t i = 0; i < states.size(); ++i)
	{
		auto state = states[i]->as<ob::State>();

		Eigen::Vector3d point;
		const auto &values = state->as<ob::RealVectorStateSpace::StateType>()->values;
		point[0] = values[0];
		point[1] = values[1];
		point[2] = values[2];

		pathPoints.push_back(point);
	}

	return vectorMsg(pathPoints);
}

nav_msgs::Path Planner::vectorMsg(const std::vector<Eigen::Vector3d> &pathVector)
{
	nav_msgs::Path msg;

	msg.header.frame_id = "world";
	msg.poses.resize(pathVector.size());
	for (int i = 0; i < pathVector.size(); i++)
	{
		msg.poses[i].pose.position.x = pathVector[i][0];
		msg.poses[i].pose.position.y = pathVector[i][1];
		msg.poses[i].pose.position.z = pathVector[i][2];
	}

	return msg;
}

// bool Planner::replan(void)
// {
// 	if(path_smooth != NULL)
// 	{
// 		og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
// 		ROS_INFO("Total Points:" << path->getStateCount ());
// 		double distance;
// 		if(pdef->hasApproximateSolution())
// 		{
// 			DBG("Goal state not satisfied and distance to goal is: " << pdef->getSolutionDifference());
// 			replan_flag = true;
// 		}
// 		else
// 		{
// 			for (std::size_t idx = 0; idx < path->getStateCount (); idx++)
// 			{
// 				if(!replan_flag)
// 				{
// 					replan_flag = !isStateValid(path->getState(idx));
// 				}
// 				else
// 					break;
// 			}
// 		}
// 	}
// 	if(replan_flag)
// 	{
// 		pdef->clearSolutionPaths();
// 		DBG("Replanning");
// 		plan();
// 		return true;
// 	}
// 	else
// 	{
// 		DBG("Replanning not required");
// 		return false;
// 	}
// }

void Planner::plan(void)
{

	// attempt to solve the problem within X seconds of planning time
	// Change in X seconds of planning time deeply affects the determined path
	// It is quite possible that planner will result in different paths for different planning time
	ob::PlannerStatus solved = o_plan->solve(3);

	if (solved)
	{
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		ROS_INFO("Found solution:");
		auto path = pdef->getSolutionPath();
		og::PathGeometric *pathGeo = path->as<og::PathGeometric>();
		pathGeo->printAsMatrix(std::cout);
		planned_path_pub.publish(pathMsg(pathGeo));
		o_plan->clear();

		// Path smoothing using bspline
		og::PathSimplifier *pathBSpline = new og::PathSimplifier(si);
		path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric &>(*pdef->getSolutionPath()));
		pathBSpline->smoothBSpline(*path_smooth);
		// pathBSpline->collapseCloseVertices(*path_smooth);

		// replan_flag = false;
	}
	else
		ROS_ERROR("No solution found");
}

bool Planner::isStateValid(const ob::State *state)
{
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>();

	// check validity of state defined by pos
	fcl::Vector3<double> translation(pos->values[0], pos->values[1], pos->values[2]);
	// ROS_INFO("State: " << translation);
	aircraftObject->setTranslation(translation);
	fcl::CollisionRequest<double> requestType;
	fcl::CollisionResult<double> collisionResult;

	if (treeObj && aircraftObject)
	{
		fcl::collide(aircraftObject.get(), treeObj.get(), requestType, collisionResult);
		return (!collisionResult.isCollision());
	}
	return false;
}

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.

ob::OptimizationObjectivePtr Planner::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr &si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	// obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}

// std::vector<std::tuple<double, double, double>> Planner::getSmoothPath()
// {
// 	std::vector<std::tuple<double, double, double>> path;
// 	for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
// 	{
// 		// cast the abstract state type to the type we expect
// 		const ob::RealVectorStateSpace::StateType *pos = path_smooth->getState(idx)->as<ob::RealVectorStateSpace::StateType>();
// 		path.push_back(std::tuple<double, double, double>(pos->values[0], pos->values[1], pos->values[2]));
// 	}
// 	return path;
// }

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Planner");
	Planner n;
	ros::spin();
	return 0;
}