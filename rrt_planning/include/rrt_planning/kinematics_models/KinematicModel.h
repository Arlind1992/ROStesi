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

#ifndef INCLUDE_KINEMATICS_MODELS_KINEMATICMODEL_H_
#define INCLUDE_KINEMATICS_MODELS_KINEMATICMODEL_H_

#include <Eigen/Dense>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>

#include "rrt_planning/map/Bounds.h"

namespace rrt_planning
{

class KinematicModel
{
public:
    typedef Eigen::VectorXd state_type;

    virtual Eigen::VectorXd compute(const Eigen::VectorXd& x0, const Eigen::VectorXd& u, double delta) = 0;
    virtual Eigen::VectorXd applyTransform(const Eigen::VectorXd& x0, const Eigen::VectorXd& T) = 0;
    virtual Eigen::VectorXd getInitialState() = 0;
    virtual Eigen::VectorXd getRandomState(const Bounds& bounds) = 0;
    virtual Eigen::VectorXd anyAngleSampling(std::vector<geometry_msgs::PoseStamped>& plan,
            double w, double deltaTheta) = 0;


    inline unsigned int getStateSize()
    {
        return stateSize;
    }

    inline unsigned int getActionSize()
    {
        return actionSize;
    }

    virtual ~KinematicModel()
    {

    }


protected:
    unsigned int stateSize;
    unsigned int actionSize;

    const double dt = 1e-3;

};

}

#endif /* INCLUDE_KINEMATICS_MODELS_KINEMATICMODEL_H_ */
