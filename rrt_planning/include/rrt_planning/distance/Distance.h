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

#ifndef INCLUDE_RRT_PLANNING_DISTANCE_DISTANCE_H_
#define INCLUDE_RRT_PLANNING_DISTANCE_DISTANCE_H_

#include <Eigen/Dense>

namespace rrt_planning
{

class Distance
{
public:
    virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) = 0;
    virtual ~Distance()
    {

    }
};

class L2Distance : public Distance
{
public:
    virtual double operator()(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
    {
        return (x1.head(2)-x2.head(2)).norm();
    }

};

}

#endif /* INCLUDE_RRT_PLANNING_DISTANCE_DISTANCE_H_ */
