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

#ifndef INCLUDE_RRT_PLANNING_GRID_GRID_H_
#define INCLUDE_RRT_PLANNING_GRID_GRID_H_

#include <vector>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include "rrt_planning/map/ROSMap.h"

namespace rrt_planning
{

class Grid
{
public:
    Grid();
	Grid(ROSMap* map);
	double cost(std::pair<int, int> s, std::pair<int, int> s_next);
	double heuristic(std::pair<int, int> s, std::pair<int, int> s_next);
	bool lineOfSight(std::pair<int, int> s, std::pair<int, int> s_next);
	std::vector<std::pair<int, int>> getNeighbors(std::pair<int, int> s);
	std::pair<int, int> convertPose(const geometry_msgs::PoseStamped& msg);

private:
    double gridResolution;	// Cell edges in meters
	std::vector< std::vector<unsigned char> > grid;
};

}

#endif /* INCLUDE_RRT_PLANNING_GRID_GRID_H_ */