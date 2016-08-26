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

#include <iostream>

#include "rrt_planning/kinematics_models/DifferentialDrive.h"

using namespace rrt_planning;

int main(int argc, char *argv[])
{

	Eigen::VectorXd x0(3);
	x0 << 0, 0, 0;

	Eigen::VectorXd u(2);
	u << 1.0, 0.01;

	DifferentialDrive model;

	Eigen::VectorXd xf = model.compute(x0, u, 5.0);
	std::cout << xf << std::endl;

}
