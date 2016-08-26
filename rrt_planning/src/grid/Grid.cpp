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

using namespace std;

namespace rrt_planning
{
Grid::Grid()
{
}


Grid::Grid(ROSMap* map)
{
    //TODO
}


vector<pair<int, int>> Grid::getNeighbors(pair<int, int> s)
{
    //TODO
}


double Grid::cost(pair<int, int> s, pair<int, int> s_next)
{
    //TODO
}


double Grid::heuristic(pair<int, int> s, pair<int, int> s_next)
{
    //TODO
}


bool Grid::lineOfSight(pair<int, int> s, pair<int, int> s_next)
{
    //TODO
}


std::pair<int, int> Grid::convertPose(const geometry_msgs::PoseStamped& msg)
{
    auto& t_ros = msg.pose.position;

    //TODO convert to grid indices

    return make_pair(t_ros.x, t_ros.y);
}

}
