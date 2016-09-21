/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Alessandro Riva
 * Versione 1.0
 *
 * This file is part of rrt_planning.
 *
 * rrt_planning is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rrt_planning is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rrt_planning.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rrt_planning/ThetaStarPlanner.h"

#include <pluginlib/class_list_macros.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planning::ThetaStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;
using namespace Eigen;

//Default Constructor
namespace rrt_planning
{

ThetaStarPlanner::ThetaStarPlanner()
{
    grid = nullptr;
	map = nullptr;
}

ThetaStarPlanner::ThetaStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}


void ThetaStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	double discretization;

    //Get parameters from ros parameter server
    ros::NodeHandle private_nh("~/" + name);
	private_nh.param("discretization", discretization, 0.1);

	map = new ROSMap(costmap_ros);
    grid = new Grid(*map, discretization);
	
}

bool ThetaStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{
	clearInstance();

    s_start = grid->convertPose(start);
    s_goal = grid->convertPose(goal);
	pair<int, int> s_null = make_pair(-1,-1);

	if(!grid->isFree(s_start))
	{
		cout << "Invalid starting position" << endl;
		return true;
	}

	if(!grid->isFree(s_goal))
	{
		cout << "Invalid target position" << endl;
		return true;
	}

    //Init variables
    g[s_start] = 0.0;
    parent[s_start] = s_start;
	insertFrontierNode(s_start, 0.0);

    ROS_INFO("Planner started");

    while(!open.empty())
    {
		//Pop the best frontier node
		FrontierNode* f = *open.begin();
        auto s = f->getNode();
		removeFrontierNode(s);
		closed.insert(s);

        if(s == s_goal) break;

        for(auto&& s_next: grid->getNeighbors(s))
            if(closed.count(s_next) == 0)
            {
                if(openMap.count(s_next) == 0)
                {
                    g[s_next] = std::numeric_limits<double>::infinity();
                    parent[s_next] = s_null;
                }
                updateVertex(s, s_next);
            }
    }

	//Publish plan
	vector<VectorXd> path;
	auto state = s_goal;
	path.push_back(grid->toMapPose(state.first, state.second));
	do
	{
		state = parent[state];

		if(state == s_null)
		{
			cout << "Invalid plan";
			return true;
		}

		path.push_back(grid->toMapPose(state.first, state.second));
	}while(state != s_start);

	reverse(path.begin(), path.end());

	publishPlan(path, plan, start.header.stamp);

    return true;
}


void ThetaStarPlanner::updateVertex(pair<int, int> s, pair<int, int> s_next)
{
    auto g_old = g[s_next];

    computeCost(s, s_next);

    if(g[s_next] < g_old)
    {
        if(openMap.count(s_next) != 0) removeFrontierNode(s_next);

		double frontierCost =  g[s_next] + grid->heuristic(s_next, s_goal);

        insertFrontierNode(s_next, frontierCost);
    }
}


void ThetaStarPlanner::computeCost(pair<int, int> s, pair<int, int> s_next)
{
	//cout << "Considero: " << s_next.first << "-" << s_next.second << " da " << s.first << "-" << s.second << endl;

	if(grid->lineOfSight(parent[s], s_next))
	{
		//cout << " > sono in line of sight con " << parent[s].first << "-" << parent[s].second << " (ls " << (g[parent[s]] + grid->cost(parent[s], s_next)) << " vs g[s_next] " << g[s_next] << ")" << endl;
        //Path 2
        if(g[parent[s]] + grid->cost(parent[s], s_next) <= g[s_next])
        {
            g[s_next] = g[parent[s]] + grid->cost(parent[s], s_next);
            parent[s_next] = parent[s];
			//cout << " > Parent di " << s_next.first << "-" << s_next.second << " aggiornato con " << parent[s].first << "-" << parent[s].second << endl;
        }
	}
    else
	{
		//cout << " > NON sono in lof con " << parent[s].first << "-" << parent[s].second << " (ls " << (g[parent[s]] + grid->cost(parent[s], s_next)) << " vs g[s_next] " << g[s_next] << ")";
    	//Path 1
        if(g[s] + grid->cost(s, s_next) < g[s_next])
        {
			//cout << " UPDATED";
	        g[s_next] = g[s] + grid->cost(s, s_next);
            parent[s_next] = s;
        }
		//cout <<endl;
	}
}


void ThetaStarPlanner::insertFrontierNode(pair<int, int> s, double cost)
{
	FrontierNode *frontierNode = new FrontierNode(s, cost);
    open.insert(frontierNode);
	openMap[s] = frontierNode;
}


bool ThetaStarPlanner::removeFrontierNode(pair<int, int> s)
{
	FrontierNode* f = openMap[s];

	open.erase(f);
	openMap.erase(s);

	delete f;

	return true;
}


void ThetaStarPlanner::publishPlan(std::vector<VectorXd>& path,
                             std::vector<geometry_msgs::PoseStamped>& plan, const ros::Time& stamp)
{
    for(auto x : path)
    {
        geometry_msgs::PoseStamped msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = "map";

        msg.pose.position.x = x(0);
        msg.pose.position.y = x(1);
        msg.pose.position.z = 0;

        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = 0;
        msg.pose.orientation.w = 1;

        plan.push_back(msg);
    }
}


void ThetaStarPlanner::clearInstance()
{
	open.clear();
	openMap.clear(); 
	closed.clear();
	parent.clear();
	g.clear();
}


ThetaStarPlanner::~ThetaStarPlanner()
{
	if(grid)
		delete grid;

	if(map)
		delete map;

}


};
