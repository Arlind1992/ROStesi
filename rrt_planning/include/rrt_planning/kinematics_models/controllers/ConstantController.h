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

#ifndef INCLUDE_RRT_PLANNING_KINEMATICS_MODELS_CONTROLLERS_CONSTANTCONTROLLER_H_
#define INCLUDE_RRT_PLANNING_KINEMATICS_MODELS_CONTROLLERS_CONSTANTCONTROLLER_H_

namespace rrt_planning
{

class ConstantController : public Controller
{
public:
    virtual inline Eigen::VectorXd operator()(const Eigen::VectorXd& x0) const override
    {
        return u;
    }


    inline void setControl(const Eigen::VectorXd& u)
    {
        this->u = u;
    }

    virtual ~ConstantController()
    {

    }

private:
    Eigen::VectorXd u;

};


}


#endif /* INCLUDE_RRT_PLANNING_KINEMATICS_MODELS_CONTROLLERS_CONSTANTCONTROLLER_H_ */
