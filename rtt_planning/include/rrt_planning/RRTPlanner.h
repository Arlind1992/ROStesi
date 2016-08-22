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

#ifndef INCLUDE_RTTPLANNER_H_
#define INCLUDE_RTTPLANNER_H_

/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

#include "rrt_planning/distance/Distance.h"
#include "rrt_planning/kinematics_models/KinematicModel.h"

namespace rrt_planning
{

class RRTPlanner : public nav_core::BaseGlobalPlanner
{
public:

    RRTPlanner();
    RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    Eigen::VectorXd randomState();

    bool newState(const Eigen::VectorXd& xRand,
                  const Eigen::VectorXd& xNear,
                  Eigen::VectorXd& xNew);

    Eigen::VectorXd convertPose(const geometry_msgs::PoseStamped& pose);

    void publishSegment(const Eigen::VectorXd& xStart, const Eigen::VectorXd& xEnd);


private:
    costmap_2d::Costmap2DROS* costmap;

    int K;
    double maxX;
    double maxY;
    double minX;
    double minY;
    double deltaT;
    double deltaX;
    double greedy;

    KinematicModel* kinematicModel;
    Distance* distance;

    //Visualization of rtt
    ros::Publisher vis_pub;

    //TODO move elsewhere
    double maxU1;
    double maxU2;
    double minU1;
    double minU2;
    int discretization;
};
};



#endif /* INCLUDE_RTTPLANNER_H_ */
