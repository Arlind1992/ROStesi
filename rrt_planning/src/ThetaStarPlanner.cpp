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


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planning::ThetaStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

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
	open.clear();
	openMap.clear(); 
	closed.clear();
	parent.clear();
	g.clear();

    s_start = grid->convertPose(start);
    s_goal = grid->convertPose(goal);

	cout << "START " << s_start.first << ", " << s_start.second << endl;
	cout << "GOAL " << s_goal.first << ", " << s_goal.second << endl;	

    //Init variables
    g[s_start] = 0.0;
    parent[s_start] = s_start;
	insertFrontierNode(s_start, 0.0);
    

    ROS_INFO("Planner started");

	int t = 0; 					//Debug

    while(!open.empty())
    {
		//Get the best frontier node
		FrontierNode* f = *open.begin();
        auto s = f->getNode();

		cout << "CELL " << s.first << ", " << s.second << endl;

		t++;					//Debug
		if(t>30) break;			//Debug

        if(s == s_goal) break;

        closed.insert(s);

        for(auto&& s_next: grid->getNeighbors(s))
            if(closed.find(s_next) == closed.end())
            {
                if(openMap.find(s_next) == openMap.end())
                {
                    g[s_next] = std::numeric_limits<double>::infinity();
                    //parent[s_next] = ;
                }
                updateVertex(s, s_next);
            }
    }

	//Print path
	/*auto state = s_goal;
	cout << state.first << ", " << state.second << ", ";

	do
	{
		state = parent[state];
		cout << state.first << ", " << state.second << ", ";

	}while(state == s_start);*/


    //TODO convert plan and publish

	plan.push_back(start);
	plan.push_back(goal);

    return true;
}


void ThetaStarPlanner::updateVertex(pair<int, int> s, pair<int, int> s_next)
{
    auto g_old = g[s_next];

    computeCost(s, s_next);

    if(g[s_next] < g_old)
    {
        if(openMap.find(s_next) != openMap.end()) removeFrontierNode(s_next);

		int frontierCost =  g[s_next] + grid->heuristic(s_next, s_goal);

        insertFrontierNode(s_next, frontierCost);
    }
}


void ThetaStarPlanner::computeCost(pair<int, int> s, pair<int, int> s_next)
{
    if(grid->lineOfSight(parent[s], s_next))
	{
        /* Path 2 */
        if(g[parent[s]] + grid->cost(parent[s], s_next) < g[s_next])
        {
            g[s_next] = g[parent[s]] + grid->cost(parent[s], s_next);
            parent[s_next] = parent[s];
        }
	}
    else
	{
    	/* Path 1 */
        if(g[s] + grid->cost(s, s_next) < g[s_next])
        {
	        g[s_next] = g[s] + grid->cost(s, s_next);
            parent[s_next] = s;
        }
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


ThetaStarPlanner::~ThetaStarPlanner()
{
	if(grid)
		delete grid;

	if(map)
		delete map;

}


};
