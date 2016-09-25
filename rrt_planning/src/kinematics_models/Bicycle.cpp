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
#include <angles/angles.h>

using namespace Eigen;
using namespace std;

namespace rrt_planning
{

Bicycle::Bicycle(Controller& controller) : KinematicModel(controller)
{
    stateSize = 4;
    actionSize = 2;

    deltaPhi = 2*M_PI/3;
    l = 0.2;
}

VectorXd Bicycle::compute(const VectorXd& x0, double delta)
{
    boost::numeric::odeint::runge_kutta4<state_type,double,state_type,double,
			boost::numeric::odeint::vector_space_algebra> stepper;
    Eigen::VectorXd x = x0;
    boost::numeric::odeint::integrate_const(stepper, *this, x, 0.0, delta, dt);

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

VectorXd Bicycle::getInitialState()
{
    return VectorXd::Zero(4);
}

VectorXd Bicycle::getRandomState(const Bounds& bounds)
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


VectorXd Bicycle::anyAngleSampling(vector<geometry_msgs::PoseStamped>& plan,
                                   double width, double deltaTheta)
{
    //TODO implement!
    return VectorXd::Zero(4);
}

Eigen::VectorXd Bicycle::computeControl(const Eigen::VectorXd& x)
{
	VectorXd u = controller(x);

	// Saturation of the steering wheel
	double maxOmega = angles::shortest_angular_distance(copysign(deltaPhi, u(1)), x(2))/dt;

	u(1) = abs(u(1)) > abs(maxOmega) ? maxOmega : u(1);

	return u;
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

    //FIXME
    //dx = A*u;

}

}
