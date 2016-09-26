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

#include <iostream>

#include "rrt_planning/kinematics_models/DifferentialDrive.h"
#include "rrt_planning/kinematics_models/controllers/POSQ.h"

using namespace rrt_planning;

int main(int argc, char** argv)
{
	POSQ controller(1.0, 1.0, 1.0, 1.0);
	DifferentialDrive kinematic(controller);

	Eigen::VectorXd start(3);
	start << 0, 0, 0;

	Eigen::VectorXd goal(3);
	goal << 0, 0, M_PI/3;

	controller.setGoal(goal);

	auto result = kinematic.compute(start, 100);

	std::cout << result.transpose() << std::endl;

}
