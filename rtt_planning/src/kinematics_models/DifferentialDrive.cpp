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

#include "rtt_planning/kinematics_models/DifferentialDrive.h"
//#include "rtt_planning/utils/EigenOdeint.h"
#include <boost/numeric/odeint.hpp>

using namespace Eigen;
using namespace boost::numeric::odeint;

Eigen::VectorXd DifferentialDrive::operator()(const VectorXd& x, const VectorXd& u)
{
	this->u = u;

	runge_kutta4<state_type,double,state_type,double,vector_space_algebra> stepper;
	double t0 = 0;
	double t1 = 1.0;
	double dt = 1e-3;
	VectorXd x0 = x;
	integrate_const(stepper, *this, x0, t0, t1, dt);
}


void DifferentialDrive::operator()(const state_type& x, state_type& dx,
	                        const double /* t */)
{
	Matrix<double, 3, 3> A;

	double theta = x(2);

	A << sin(theta), 0,
	     cos(theta), 0,
		          0, 1;

	dx = A*u;

}
