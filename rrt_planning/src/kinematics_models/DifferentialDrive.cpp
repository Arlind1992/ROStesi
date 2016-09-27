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

#include "rrt_planning/kinematics_models/DifferentialDrive.h"
#include "rrt_planning/utils/RandomGenerator.h"
#include <boost/numeric/odeint.hpp>


using namespace Eigen;
using namespace std;

namespace rrt_planning
{

DifferentialDrive::DifferentialDrive(Controller& controller) : KinematicModel(controller)
{
    stateSize = 3;
    actionSize = 2;
}

VectorXd DifferentialDrive::compute(const VectorXd& x0, double delta)
{
    boost::numeric::odeint::runge_kutta4<state_type,double,state_type,double,
			boost::numeric::odeint::vector_space_algebra> stepper;
    Eigen::VectorXd x = x0;
    boost::numeric::odeint::integrate_const(stepper, *this, x, 0.0, delta, dt);

    return x;
}

Eigen::VectorXd DifferentialDrive::getInitialState()
{
    return VectorXd::Zero(3);
}

Eigen::VectorXd DifferentialDrive::getRandomState(const Bounds& bounds)
{
    VectorXd xRand;
    xRand.setRandom(3);

    xRand += VectorXd::Ones(3);
    xRand /= 2;

    xRand(0) *= bounds.maxX - bounds.minX;
    xRand(1) *= bounds.maxY - bounds.minY;
    xRand(2) *= 2*M_PI;

    xRand(0) += bounds.minX;
    xRand(1) += bounds.minY;
    xRand(2) += -M_PI;

    return xRand;
}


void DifferentialDrive::operator()(const state_type& x, state_type& dx,
                                   const double /* t */)
{
    Matrix<double, 3, 2> A;

    double theta = x(2);

    A << cos(theta), 0,
    sin(theta), 0,
    0, 1;

    dx = A*controller(x);

}

}
