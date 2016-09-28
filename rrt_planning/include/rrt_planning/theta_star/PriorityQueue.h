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

#ifndef INCLUDE_RRT_PLANNING_GRID_PRIORITYQUEUE_H_
#define INCLUDE_RRT_PLANNING_GRID_PRIORITYQUEUE_H_

#include "rrt_planning/theta_star/FrontierNode.h"

namespace rrt_planning
{

class PriorityQueue
{
public:
	void insert(Cell cell, double cost);
	void remove(Cell cell);
	bool contains(Cell cell);
	bool empty();
	Cell pop();
	void clear();


private:
    struct Cmp
    {
        bool operator()(FrontierNode* a, FrontierNode* b)
        {
            return a->getCost() < b->getCost();
        }
    };

    std::set<FrontierNode*, Cmp> open;
    std::map<std::pair<int, int>, FrontierNode*> openMap;
};

}


#endif /* INCLUDE_RRT_PLANNING_GRID_PRIORITYQUEUE_H_ */
