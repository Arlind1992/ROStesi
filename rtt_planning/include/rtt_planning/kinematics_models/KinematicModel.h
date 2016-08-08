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

#ifndef INCLUDE_KINEMATICS_MODELS_KINEMATICMODEL_H_
#define INCLUDE_KINEMATICS_MODELS_KINEMATICMODEL_H_

#include <Eigen/Dense>

class KinematicModel
{
public:
    typedef Eigen::VectorXd state_type;

    virtual Eigen::VectorXd compute(const Eigen::VectorXd& x0, const Eigen::VectorXd& u, double delta) = 0;

    virtual ~KinematicModel()
    {

    }

protected:
    const double dt = 1e-1;



};


#endif /* INCLUDE_KINEMATICS_MODELS_KINEMATICMODEL_H_ */
