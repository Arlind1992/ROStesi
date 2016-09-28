/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Davide Tateo
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

#ifndef INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_
#define INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

#include "rrt_planning/map/ROSMap.h"
#include "rrt_planning/grid/Grid.h"
#include "rrt_planning/theta_star/FrontierNode.h"
#include "rrt_planning/visualization/Visualizer.h"


namespace rrt_planning
{

class ThetaStarPlanner : public nav_core::BaseGlobalPlanner
{

public:
    ThetaStarPlanner();
    ThetaStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

    ~ThetaStarPlanner();

private:
    void updateVertex(std::pair<int, int> s, std::pair<int, int> s_next);
    void computeCost(std::pair<int, int> s, std::pair<int, int> s_next);
    void insertFrontierNode(std::pair<int, int> s, double cost);
    bool removeFrontierNode(std::pair<int, int> s);
    void publishPlan(std::vector<Eigen::VectorXd>& path, std::vector<geometry_msgs::PoseStamped>& plan,
		const ros::Time& stamp, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);
    void clearInstance();
	void displayClosed();
	void displayOpen();

private:

	ros::Publisher pub;

    struct Cmp
    {
        bool operator()(FrontierNode *a, FrontierNode *b)
        {
            return a->getCost() < b->getCost();
        }
    };

	static const std::pair<int, int> S_NULL;

    ROSMap* map;
    Grid* grid;

    std::pair<int, int> s_start;
    std::pair<int, int> s_goal;

    std::map<std::pair<int, int>, double> g;
    std::map<std::pair<int, int>, std::pair<int, int>> parent;
    std::set<FrontierNode*, Cmp> open;
    std::map<std::pair<int, int>, FrontierNode*> openMap;
    std::set<std::pair<int, int>> closed;

    Visualizer visualizer;
};

}

#endif /* INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_ */
