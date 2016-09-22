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

#include "rrt_planning/kinematics_models/Bicycle.h"
#include <boost/numeric/odeint.hpp>

using namespace Eigen;
using namespace boost::numeric::odeint;

namespace rrt_planning
{

Bicycle::Bicycle()
{
    stateSize = 4;
    actionSize = 2;

    deltaPhi = 2*M_PI/3;
    l = 0.2;
}

Eigen::VectorXd Bicycle::compute(const VectorXd& x0, const VectorXd& u, double delta)
{
    this->u = u;

    double omega = u(1);
    double phi0 = x0(3);

    double tSat = ((omega >0 ? deltaPhi : -deltaPhi) -phi0) / omega;

    double deltaNormal = (tSat > delta) ? delta : tSat;
    double deltaSaturation = (tSat > delta) ? 0 : (delta - deltaNormal);

    runge_kutta4<state_type,double,state_type,double,vector_space_algebra> stepper;
    VectorXd x = x0;

    if(deltaNormal > 0)
    	integrate_const(stepper, *this, x, 0.0, delta, deltaNormal);

    this->u(1) = 0;

    if(deltaSaturation > 0)
    	integrate_const(stepper, *this, x, 0.0, delta, deltaSaturation);

    return x;
}

VectorXd Bicycle::applyTransform(const VectorXd& x0, const VectorXd& T)
{
	//TODO Implement motion primitives for bicycles in a more complex way
    VectorXd xf = x0;

    double theta = x0(2);

    xf(0) += cos(theta)*T(0) - sin(theta)*T(1);
    xf(1) += sin(theta)*T(0) + cos(theta)*T(1);
    xf(2) += T(2);

    return xf;
}

Eigen::VectorXd Bicycle::getInitialState()
{
    return VectorXd::Zero(4);
}

Eigen::VectorXd Bicycle::getRandomState(const Bounds& bounds)
{
    VectorXd xRand;
    xRand.setRandom(stateSize);

    xRand += VectorXd::Ones(stateSize);
    xRand /= 2;

    xRand(0) *= bounds.maxX - bounds.minX;
    xRand(1) *= bounds.maxY - bounds.minY;
    xRand(2) *= 2*M_PI;
    xRand(3) *= deltaPhi;

    xRand(0) += bounds.minX;
    xRand(1) += bounds.minY;
    xRand(2) += -M_PI;
    xRand(3) += -0.5*deltaPhi;

    return xRand;
}


void Bicycle::operator()(const state_type& x, state_type& dx,
                                   const double /* t */)
{
    Matrix<double, 4, 2> A;

    double theta = x(2);
    double phi = x(3);

    A << cos(theta)*cos(phi), 0,
    sin(theta)*cos(phi), 0,
	sin(phi)/l, 0,
    0, 1;

    dx = A*u;

}

}
