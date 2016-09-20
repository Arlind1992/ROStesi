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

#include "rrt_planning/grid/Grid.h"
#include <cmath>
#include <Eigen/Dense>

using namespace std;

namespace rrt_planning
{


Grid::Grid(Map& map, double gridResolution): map(map),
	gridResolution(gridResolution)
{
}


vector<pair<int, int>> Grid::getNeighbors(pair<int, int> s)
{
	//TODO eight-connected or line-of-sight?

	int X = s.first;
	int Y = s.second;

	vector<pair<int, int>> neighbors;

	//Given (X,Y), retrive all the eight-connected free cells
	for(int i = -1; i <= 1; i++)
		for(int j = -1; j <= 1; j++)
		{
			if(i == 0 && j == 0) continue;

			auto&& pos = toMapPose(X+i, Y+j);

			if(map.isFree(pos))
				neighbors.push_back(make_pair(X+i, Y+j));
		}

	return neighbors;
}


double Grid::cost(pair<int, int> s, pair<int, int> s_next)
{
	//TODO no obstacles (8-connected)?

	int X1 = s.first;
	int Y1 = s.second;

	int X2 = s_next.first;
	int Y2 = s_next.second;

	return sqrt( (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) );
}


double Grid::heuristic(pair<int, int> s, pair<int, int> s_next)
{
	int X1 = s.first;
	int Y1 = s.second;

	int X2 = s_next.first;
	int Y2 = s_next.second;

	return sqrt( (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) );
}


bool Grid::lineOfSight(pair<int, int> s, pair<int, int> s_next)
{
	int X1 = s.first;
	int Y1 = s.second;

	int X2 = s_next.first;
	int Y2 = s_next.second;

	//Determine how steep the line is
	bool is_steep = abs(Y2-Y1) > abs(X2-X1);

	//Possibly rotate the line
	if(is_steep)
	{
		swap(X1, Y1);
		swap(X2, Y2);
	}

	//Possibly swap start and end
	bool swapped = false;

	if(X1 > X2)
	{
		swap(X1, X2);
		swap(Y1, Y2);
	}

	int error = (X2 - X1) / 2;
	int ystep = Y1 < Y2 ? 1 : -1;

	int y = Y1;

	//Check for obstalces through the line
	for(int x = X1; x <= X2; x++)
	{
		Eigen::VectorXd pos;
	
		if(is_steep)
			pos = toMapPose(y, x);
		else
			pos = toMapPose(x, y);

		if(!map.isFree(pos)) return false;

		error -= abs(Y2 - Y1);
		if(error < 0)
		{
			y += ystep;
			error += (X2 - X1);
		}
	}

	return true;
}


std::pair<int, int> Grid::convertPose(const geometry_msgs::PoseStamped& msg)
{
    auto& t_ros = msg.pose.position;

	Bounds bounds = map.getBounds();

	int X_index = floor( (t_ros.x - bounds.minX) / gridResolution );
	int Y_index = floor( (t_ros.y - bounds.minY) / gridResolution );

    return make_pair(X_index, Y_index);
}


Eigen::VectorXd Grid::toMapPose(int X, int Y)
{
	Bounds bounds = map.getBounds();

	Eigen::VectorXd pos(2);

	pos(0) = (0.5 + X) * gridResolution + bounds.minX;
	pos(1) = (0.5 + Y) * gridResolution + bounds.minY;
	
    return pos;
}


bool Grid::isFree(pair<int, int> s)
{
	Eigen::VectorXd pos;
	pos = toMapPose(s.first, s.second);

	return map.isFree(pos);
}

}
