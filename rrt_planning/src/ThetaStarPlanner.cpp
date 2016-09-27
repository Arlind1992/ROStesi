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
    private_nh.param("discretization", discretization, 0.2);

    map = new ROSMap(costmap_ros);
    grid = new Grid(*map, discretization);

    visualizer.initialize(private_nh);
}

bool ThetaStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{
    clearInstance();
    visualizer.clean();

    //Init the position of the special states
    s_start = grid->convertPose(start);
    s_goal = grid->convertPose(goal);
    pair<int, int> s_null = make_pair(-1,-1);

    //Test starting position
    if(!grid->isFree(s_start))
    {
        ROS_INFO("Invalid starting position");
        return false;
    }

    //Test target position
    if(!grid->isFree(s_goal))
    {
        ROS_INFO("Invalid target position");
        return false;
    }

    //Init variables
    g[s_start] = 0.0;
    parent[s_start] = s_start;
    insertFrontierNode(s_start, 0.0);

    ROS_INFO("Planner started");

    //Compute plan
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
            ROS_INFO("Invalid plan");
            return false;
        }

        path.push_back(grid->toMapPose(state.first, state.second));
    }
    while(state != s_start);

    reverse(path.begin(), path.end());
    publishPlan(path, plan, start.header.stamp, start, goal);
    visualizer.displayPlan(plan);

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

    if(grid->lineOfSight(parent[s], s_next))
    {
        //Path 2
        if(g[parent[s]] + grid->cost(parent[s], s_next) <= g[s_next])
        {
            g[s_next] = g[parent[s]] + grid->cost(parent[s], s_next);
            parent[s_next] = parent[s];
        }
    }
    else
    {
        //Path 1
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


void ThetaStarPlanner::publishPlan(std::vector<Eigen::VectorXd>& path, std::vector<geometry_msgs::PoseStamped>& plan,
		const ros::Time& stamp, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
{
	plan.push_back(start);

    for(int i = 1; i < path.size() - 1; i++)
    {
		auto&& p1 = path[i];
		auto&& p2 = path[i+1];

        geometry_msgs::PoseStamped msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = "map";

        msg.pose.position.x = p1(0);
        msg.pose.position.y = p1(1);
        msg.pose.position.z = 0;

		double angle = atan2(p2(1) - p1(1), p2(0) - p1(0));

		Matrix3d m;
        m = AngleAxisd(angle, Vector3d::UnitZ())
            * AngleAxisd(0, Vector3d::UnitY())
            * AngleAxisd(0, Vector3d::UnitX());

        Quaterniond q(m);

        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        plan.push_back(msg);
    }
	
	plan.push_back(goal);
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
	for(auto f: open)
		removeFrontierNode(f->getNode());

    if(grid)
        delete grid;

    if(map)
        delete map;
}


};
