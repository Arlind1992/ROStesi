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

#include <pluginlib/class_list_macros.h>
#include "rtt_planning/RTTPlanner.h"

#include "rtt_planning/rtt/RTT.h"
#include "rtt_planning/kinematics_models/DifferentialDrive.h"

using namespace Eigen;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rtt_planning::RTTPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rtt_planning
{

RTTPlanner::RTTPlanner ()
{
    costmap = nullptr;

    K = 0;
    maxX = 0;
    maxY = 0;
    minX = 0;
    minY = 0;
}

RTTPlanner::RTTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}


void RTTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    this->costmap = costmap_ros;

    //Get parameters from ros parameter server
    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("iterations", K, 2000);

    private_nh.param("maxX", maxX, 50.0);
    private_nh.param("maxY", maxY, 50.0);
    private_nh.param("minX", minX, -50.0);
    private_nh.param("minY", minY, -50.0);
}

bool RTTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan)
{
	auto& q_ros = start.pose.orientation;
	auto& t_ros = start.pose.position;

	Quaterniond q(q_ros.w, q_ros.x, q_ros.y, q_ros.z);

	Vector3d theta = q.matrix().eulerAngles(0, 1, 2);

    VectorXd x0(3);
    x0 << t_ros.x, t_ros.y, theta(3);

    L2Distance distance;
    RTT rtt(distance, x0);

    DifferentialDrive kinematicModel;

    for(unsigned int i = 0; i < K; i++)
    {
    	auto&& xRand = randomState();

    	auto* node = rtt.searchNearestNode(xRand);

    	VectorXd xNew;

    	if(newState(xRand, node->x, xNew))
    	{
    		rtt.addNode(node, xNew);
    	}

    }


    return false;

}

VectorXd RTTPlanner::randomState()
{
	VectorXd xRand;
	xRand.setRandom(3);

	xRand += VectorXd::Ones(3);
	xRand /= 2;

	xRand(0) *= maxX - minX;
	xRand(1) *= maxY - minY;
	xRand(2) *= 2*M_PI;

	xRand(0) += minX;
	xRand(1) += minY;
	xRand(2) += -M_PI;

	return xRand;
}

bool RTTPlanner::newState(const VectorXd& xNear,
    			 const VectorXd& xNew,
				 VectorXd& u)
{


	return true;
}


};
