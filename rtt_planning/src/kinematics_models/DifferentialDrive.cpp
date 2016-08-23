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

#include "rrt_planning/kinematics_models/DifferentialDrive.h"
#include <boost/numeric/odeint.hpp>

using namespace Eigen;
using namespace boost::numeric::odeint;

namespace rrt_planning
{

DifferentialDrive::DifferentialDrive(Map& map) : KinematicModel(map)
{

}

Eigen::VectorXd DifferentialDrive::compute(const VectorXd& x0, const VectorXd& u, double delta)
{
    this->u = u;

    runge_kutta4<state_type,double,state_type,double,vector_space_algebra> stepper;
    VectorXd x = x0;
    integrate_const(stepper, *this, x, 0.0, delta, dt);

    if(map.isFree(x))
        return x;
    else
    {
    	std::cerr << "Not free" << std::endl;
        throw std::exception();
    }
}


void DifferentialDrive::operator()(const state_type& x, state_type& dx,
                                   const double /* t */)
{
    Matrix<double, 3, 2> A;

    double theta = x(2);

    A << sin(theta), 0,
    cos(theta), 0,
    0, 1;

    dx = A*u;

}

}
