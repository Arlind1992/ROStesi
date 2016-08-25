/*
 * rtt_planning,
 *
 *
 * Copyright (C) 2016 Davide Tateo
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

#ifndef INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_
#define INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include "rrt_planning/grid/Grid.h"


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

private:
	void updateVertex(std::pair<int, int> s, std::pair<int, int> s_next);
	void computeCost(std::pair<int, int> s, std::pair<int, int> s_next);

private:

	template<typename T, typename priority_t>
	struct PriorityQueue {
	  typedef std::pair<priority_t, T> PQElement;
	  std::priority_queue<PQElement, std::vector<PQElement>, 
		             std::greater<PQElement>> elements;

	  inline bool empty() const { return elements.empty(); }

	  inline void put(T item, priority_t priority) {
		elements.emplace(priority, item);
	  }

	  inline T get() {
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	  }
    };


	Grid* grid;

	std::pair<int, int> s_start;
	std::pair<int, int> s_goal;

	std::map<std::pair<int, int>, double> g;
	std::map<std::pair<int, int>, std::pair<int, int>> parent;
	PriorityQueue<std::pair<int, int>, double> open;
	std::set<std::pair<int, int>> closed;

};

}

#endif /* INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_ */
