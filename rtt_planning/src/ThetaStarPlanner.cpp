/*
 * rtt_planning,
 *
 *
 * Copyright (C) 2016 Alessandro Riva
 * Versione 1.0
 *
 * This file is part of rtt_planning.
 *
 * rtt_planning is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rtt_planning is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rtt_planning.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rrt_planning/ThetaStarPlanner.h"

#include <pluginlib/class_list_macros.h>

using namespace Eigen;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planning::ThetaStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rrt_planning
{

ThetaStarPlanner::ThetaStarPlanner ()
{
	map = nullptr;
}

ThetaStarPlanner::ThetaStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	initialize(name, costmap_ros);
}


void ThetaStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	map = new ROSMap(costmap_ros);

	//Get parameters from ros parameter server
	ros::NodeHandle private_nh("~/" + name);

	//TODO convert map to grid
}

bool ThetaStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{

	Vector2i&& s_start = convertPose(start);
	Vector2i&& s_goal = convertPose(goal);

	//Init variables
	g[s_start] = 0;
	partent[s_start] = s_start;
	open.push(s_start, 0);
	closed.clear();

	ROS_INFO("Planner started");

	while(!open.empty())
	{
		auto s = open.pop();
		if(s == s_goal)
			break;

		closed.insert(s);

		for(Vector2i s_next in getNeighbors(s))
			if(closed.find(s_next) == closed.end())
			{
				if(open.find(s_next) == open.end())
				{
					g[s_next] = std::numeric_limits<double>::infinity();
					parent[s_next] = NULL;
				}
				updateVertex(s, s_next);
			}
	}

	//TODO convert plan and publish

    return false;
}


Vector2i ThetaStarPlanner::convertPose(const geometry_msgs::PoseStamped& msg)
{
    auto& t_ros = msg.pose.position;

	//TODO convert to grid indices
    x << t_ros.x, t_ros.y;

    return x;
}


vector<Vector2i> ThetaStarPlanner::getNeighbors(Vector2i s)
{
	//TODO
}


void ThetaStarPlanner::updateVertex(Vector2i s, Vector2i s_next)
{
	auto g_old = g[s_next];

	computeCost(s, s_next);

	if(g[s_next] < g_old)
	{
		if(open.find(s_next) != open.end()) open.remove(s_next);
		
		open.push(s_next, g[s_next] + AStarHeuristic(s_next));
	}
}


void ThetaStarPlanner::computeCost(Vector2i s, Vector2i s_next)
{
	if(lineOfSight(parent[s], s_next))
		/* Path 2 */
		if(g[parent[s]] + computeCost(parent[s], s_next) < g[s_next])
		{
			g[s_next] = g[parent[s]] + computeCost(parent[s], s_next);
			parent[s_next] = parent[s];
		}
	else
		/* Path 1 */
		if(g[s] + computeCost(s, s_next) < g[s_next])
		{
			g[s_next] = g[s] + computeCost(s, s_next);
			parent[s_next] = s;
		}
}


double ThetaStarPlanner::AStarHeuristic(Vector2i s)
{
	//TODO
}

};




