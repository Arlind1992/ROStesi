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

#include "rrt_planning/extenders/ExtenderFactory.h"

#include "rrt_planning/kinematics_models/DifferentialDrive.h"
#include "rrt_planning/kinematics_models/Bicycle.h"

#include "rrt_planning/kinematics_models/controllers/CostantController.h"
#include "rrt_planning/kinematics_models/controllers/POSQ.h"

#include "rrt_planning/extenders/MotionPrimitivesExtender.h"
//#include "rrt_planning/extenders/MotionPrimitivesExtender.h"

#include <stdexcept>

namespace rrt_planning
{

void ExtenderFactory::initialize(ros::NodeHandle& nh, Map& map, Distance& distance)
{
	std::string extenderName;
	std::string kinematicModelName;

	nh.param("extender", extenderName, std::string("MotionPrimitives"));
	nh.param("kinematicModel", kinematicModelName, std::string("DifferentialDrive"));

	if(extenderName == "MotionPrimitives")
	{
		CostantController* controller = new CostantController();

		this->controller = controller;

		initializeKinematic(kinematicModelName);

		extender = new MotionPrimitivesExtender(map, distance, *kinematicModel, *controller);
	}
	else
	{
		std::string controllerName;

		nh.param("controller", controllerName, std::string("POSQ"));

		if(controllerName == "POSQ")
		{
			//FIXME use parameters
			controller = new POSQ(1.0, 1.0, 1.0, 1.0);
		}
		else if(controllerName == "Costant")
		{
			controller = new CostantController();
		}

		//TODO create extender
	}

	if(!extender)
		throw std::runtime_error("Wrong parameters specified");

	extender->initialize(nh);

}

void ExtenderFactory::initializeKinematic(const std::string& kinematicModelName)
{
	if(kinematicModelName == "DifferentialDrive")
	{
		kinematicModel = new DifferentialDrive(*controller);
	}
	else if(kinematicModelName == "Bicycle")
	{
		kinematicModel = new Bicycle(*controller);
	}

}

ExtenderFactory::~ExtenderFactory()
{
	if(extender)
		delete extender;

	if(kinematicModel)
		delete kinematicModel;

	if(controller)
		delete controller;
}


}

