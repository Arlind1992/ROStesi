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

#include "rrt_planning/rrt/RRT.h"

namespace rrt_planning
{

RRT::RRT(Distance& distance, Eigen::VectorXd& x0) : distance(distance)
{
    root = new RRTNode(x0);
    nodes.push_back(root);
}

RRTNode* RRT::searchNearestNode(Eigen::VectorXd& x)
{
    RRTNode* nearest = root;
    double dMin = distance(root->x, x);

    for(auto* node : nodes)
    {
        double newDist = distance(node->x, x);

        if(newDist < dMin)
        {
            dMin = newDist;
            nearest = node;
        }
    }

    return nearest;
}

void RRT::addNode(RRTNode* parent, Eigen::VectorXd& xNew)
{
    RRTNode* child = new RRTNode(xNew);
    parent->childs.push_back(child);
    nodes.push_back(child);
}

}
