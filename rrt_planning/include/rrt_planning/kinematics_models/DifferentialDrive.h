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

#ifndef INCLUDE_KINEMATICS_MODELS_DIFFERENTIALDRIVE_H_
#define INCLUDE_KINEMATICS_MODELS_DIFFERENTIALDRIVE_H_

#include "rrt_planning/kinematics_models/KinematicModel.h"

namespace rrt_planning
{

class DifferentialDrive : public KinematicModel
{
public:
    DifferentialDrive(Controller& controller);
    virtual Eigen::VectorXd compute(const Eigen::VectorXd& x0, double delta) override;
    virtual Eigen::VectorXd getInitialState() override;
    virtual Eigen::VectorXd sampleOnBox(const Bounds& bounds) override;


    void operator()(const Eigen::VectorXd& x, Eigen::VectorXd& dx,
                    const double /* t */);

private:
    Eigen::VectorXd u;
};

}

#endif /* INCLUDE_KINEMATICS_MODELS_DIFFERENTIALDRIVE_H_ */
