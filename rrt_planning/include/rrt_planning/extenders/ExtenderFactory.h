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

#ifndef INCLUDE_RRT_PLANNING_EXTENDERS_EXTENDERFACTORY_H_
#define INCLUDE_RRT_PLANNING_EXTENDERS_EXTENDERFACTORY_H_

#include "rrt_planning/extenders/Extender.h"
#include "rrt_planning/kinematics_models/KinematicModel.h"
#include "rrt_planning/kinematics_models/controllers/Controller.h"

#include "rrt_planning/distance/Distance.h"
#include "rrt_planning/map/Map.h"

#include <ros/ros.h>

namespace rrt_planning
{

class ExtenderFactory
{
public:
    void initialize(ros::NodeHandle& nh, Map& map, Distance& distance);

    void initializeKinematic(ros::NodeHandle& nh);

    inline Extender& getExtender()
    {
        return *extender;
    }

    inline KinematicModel& getKinematicModel()
    {
        return *kinematicModel;
    }

    ~ExtenderFactory();


private:
    Extender* extender;
    KinematicModel* kinematicModel;
    Controller* controller;



};

}

#endif /* INCLUDE_RRT_PLANNING_EXTENDERS_EXTENDERFACTORY_H_ */

