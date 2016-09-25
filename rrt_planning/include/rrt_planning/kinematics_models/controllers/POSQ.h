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

#ifndef INCLUDE_RRT_PLANNING_KINEMATICS_MODELS_CONTROLLERS_POSQ_H_
#define INCLUDE_RRT_PLANNING_KINEMATICS_MODELS_CONTROLLERS_POSQ_H_

#include "rrt_planning/kinematics_models/controllers/Controller.h"

namespace rrt_planning
{

class POSQ : public Controller
{
public:
	POSQ(double Krho, double Kv, double Kalpha, double Kphi);
	virtual Eigen::VectorXd operator()(const Eigen::VectorXd& x0) const override;
	virtual ~POSQ();

private:
	const double Krho;
	const double Kv;
	const double Kalpha;
	const double Kphi;
};

}

#endif /* INCLUDE_RRT_PLANNING_KINEMATICS_MODELS_CONTROLLERS_POSQ_H_ */
