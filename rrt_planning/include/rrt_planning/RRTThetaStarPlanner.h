/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Alessandro Riva, Davide Tateo
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

#ifndef INCLUDE_RRTTHETASTARPLANNER_H_
#define INCLUDE_RRTTHETASTARPLANNER_H_

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
#include "rrt_planning/local_planner/LocalPlanner.h"

#include "rrt_planning/ThetaStarPlanner.h"


namespace rrt_planning
{

class RRTThetaStarPlanner : public nav_core::BaseGlobalPlanner
{
public:

    RRTThetaStarPlanner();
    RRTThetaStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

    virtual ~RRTThetaStarPlanner();

private:
    bool newState(const Eigen::VectorXd& xRand,
                  const Eigen::VectorXd& xNear,
                  Eigen::VectorXd& xNew);

    Eigen::VectorXd convertPose(const geometry_msgs::PoseStamped& pose);

    void publishPlan(std::vector<Eigen::VectorXd>& path, std::vector<geometry_msgs::PoseStamped>& plan,
                     const ros::Time& stamp);

    void cleanSegments();
    void publishSegment(const Eigen::VectorXd& xStart, const Eigen::VectorXd& xEnd);
	Eigen::VectorXd anyAngleSampling(std::vector<geometry_msgs::PoseStamped>& plan, double w);


private:
    Map* map;

    int K;
    double deltaX;
    double greedy;
	double laneWidth;

    KinematicModel* kinematicModel;
    Distance* distance;
    LocalPlanner* localPlanner;

	ThetaStarPlanner* thetaStarPlanner;

    //Visualization of rrt
    ros::Publisher vis_pub;

};

}



#endif /* INCLUDE_RRTTHETASTARPLANNER_H_ */
