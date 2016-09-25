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

#include "rrt_planning/kinematics_models/controllers/POSQ.h"

#include <cmath>
#include <angles/angles.h>

using namespace Eigen;
using namespace std;

namespace rrt_planning
{

POSQ::POSQ(double Krho, double Kv, double Kalpha, double Kphi) :
		Krho(Krho), Kv(Kv), Kalpha(Kalpha), Kphi(Kphi)
{

}

VectorXd POSQ::operator()(const Eigen::VectorXd& x0) const
{
	VectorXd u(2);

	double rho = (x0.head(2) - goal.head(2)).norm();
	double alpha = angles::normalize_angle(atan2(goal(1) - x0(1), goal(0) - x0(0)) - x0(2));
	double phi = angles::normalize_angle(M_PI - x0(2));

	u(0) = Krho*tanh(Kv*rho);
	u(1) = Kalpha+Kphi*phi;

	return u;
}

POSQ::~POSQ()
{

}

}
