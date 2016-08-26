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
#include <math>

using namespace std;

namespace rrt_planning
{
Grid::Grid()
{
}


Grid::Grid(ROSMap* map, double gridResolution): map(map),
	gridResolution(gridResolution)
{
	Bounds bounds = map->getBounds();

	int NX = ceil( (bounds.maxX - bounds.minX) / gridResolution );
	int NY = ceil( (bounds.maxY - bounds.minY) / gridResolution );

	VectorXd cpoint;

	for(int i = 0; i < NX; i++)
	{
		vector<unsigned char> row(NY);

		for(int j = 0; j < NY; j++)
		{
			cpoint(0) = (i + 0.5) * gridResolution;
			cpoint(1) = (j + 0.5) * gridResolution;

			grid.push_back(row);

			grid[i][j] = (this.map)->isFree(cpoint) ? 1 : 0;			
		}
	}
}


vector<pair<int, int>> Grid::getNeighbors(pair<int, int> s)
{
	//TODO eight-connected or line-of-sight?

	int X = std::get<0>(s);
	int Y = std::get<1>(s);

	vector<pair<int, int>> neighbors;

	//Given (X,Y), retrive all the eight-connected free cells
	for(int i = -1; i <= 1; i++)
		for(int j = -1; j <= 1; j++)
		{
			if(i == 0 && j == 0) continue;

			if(grid[X+i][Y+j] == 1)
				neighbors.push_back(make_pair(X+i, Y+j));
		}

	return neighbors;
}


double Grid::cost(pair<int, int> s, pair<int, int> s_next)
{
	//TODO no obstacles?

	int X1 = std::get<0>(s);
	int Y1 = std::get<1>(s);

	int X2 = std::get<0>(s_next);
	int Y2 = std::get<1>(s_next);

	return sqrt( (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) );
}


double Grid::heuristic(pair<int, int> s, pair<int, int> s_next)
{
	int X1 = std::get<0>(s);
	int Y1 = std::get<1>(s);

	int X2 = std::get<0>(s_next);
	int Y2 = std::get<1>(s_next);

	return sqrt( (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) );
}


bool Grid::lineOfSight(pair<int, int> s, pair<int, int> s_next)
{
	//TODO
}


std::pair<int, int> Grid::convertPose(const geometry_msgs::PoseStamped& msg)
{
    auto& t_ros = msg.pose.position;

	int X_index = floor( (t_ros.x - bounds.minX) / gridResolution );
	int Y_index = floor( (t_ros.Y - bounds.minY) / gridResolution );

    return make_pair(X_index, Y_index);
}

}
