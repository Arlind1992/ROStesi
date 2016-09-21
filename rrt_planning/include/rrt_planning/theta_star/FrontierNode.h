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

#ifndef INCLUDE_RRT_PLANNING_THETASTAR_FRONTIERNODE_H_
#define INCLUDE_RRT_PLANNING_THETASTAR_FRONTIERNODE_H_


namespace rrt_planning
{
class FrontierNode
{
public:
	inline FrontierNode(std::pair<int, int> node, double cost):
		node(node), cost(cost) { }

	inline std::pair<int, int> getNode() { return node; }
	inline double getCost() { return cost; }

private:
	std::pair<int, int> node;
	double cost;
};

}

#endif /* INCLUDE_RRT_PLANNING_THETASTAR_FRONTIERNODE_H_ */
