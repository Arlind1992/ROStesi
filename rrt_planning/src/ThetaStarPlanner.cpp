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
}

ThetaStarPlanner::ThetaStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}


void ThetaStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ROSMap* map = new ROSMap(costmap_ros);
    grid = new Grid(map);

    //Get parameters from ros parameter server
    ros::NodeHandle private_nh("~/" + name);
}

bool ThetaStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{
    s_start = grid->convertPose(start);
    s_goal = grid->convertPose(goal);

    //Init variables
    g[s_start] = 0.0;
    parent[s_start] = s_start;
    open.put(s_start, 0.0);
    closed.clear();

    ROS_INFO("Planner started");

    while(!open.empty())
    {
        auto s = open.get();
        if(s == s_goal)
            break;

        closed.insert(s);

        for(auto s_next: grid->getNeighbors(s))
            if(true)//closed.find(s_next) == closed.end())
            {
                if(true)//open.find(s_next) == open.end())
                {
                    g[s_next] = std::numeric_limits<double>::infinity();
                    //parent[s_next] = nullptr; //unnecessary?
                }
                updateVertex(s, s_next);
            }
    }

    //TODO convert plan and publish

    return false;
}


void ThetaStarPlanner::updateVertex(pair<int, int> s, pair<int, int> s_next)
{
    auto g_old = g[s_next];

    computeCost(s, s_next);

    if(g[s_next] < g_old)
    {
        //if(open.find(s_next) != open.end()) open.remove(s_next);

        open.put(s_next, g[s_next] + grid->heuristic(s_next, s_goal));
    }
}


void ThetaStarPlanner::computeCost(pair<int, int> s, pair<int, int> s_next)
{
    if(grid->lineOfSight(parent[s], s_next))
        /* Path 2 */
        if(g[parent[s]] + grid->cost(parent[s], s_next) < g[s_next])
        {
            g[s_next] = g[parent[s]] + grid->cost(parent[s], s_next);
            parent[s_next] = parent[s];
        }
        else
            /* Path 1 */
            if(g[s] + grid->cost(s, s_next) < g[s_next])
            {
                g[s_next] = g[s] + grid->cost(s, s_next);
                parent[s_next] = s;
            }
}

};
