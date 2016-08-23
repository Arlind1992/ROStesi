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

#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

#include "rrt_planning/RRTPlanner.h"

#include "rrt_planning/map/ROSMap.h"
#include "rrt_planning/kinematics_models/DifferentialDrive.h"
#include "rrt_planning/utils/RandomGenerator.h"
#include "rrt_planning/rrt/RRT.h"

using namespace Eigen;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planning::RRTPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rrt_planning
{

RRTPlanner::RRTPlanner ()
{
    map = nullptr;

    K = 0;
    maxX = 0;
    maxY = 0;
    minX = 0;
    minY = 0;

    deltaT = 0;
    deltaX = 0;

    greedy = 0;

    //TODO move elsewhere
    maxU1 = 0;
    maxU2 = 0;
    minU1 = 0;
    minU2 = 0;
    discretization = 0;

    kinematicModel = nullptr;
    distance = nullptr;
}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}


void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    map = new ROSMap(costmap_ros);

    //Get parameters from ros parameter server
    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("iterations", K, 2000);

    private_nh.param("maxX", maxX, 50.0);
    private_nh.param("maxY", maxY, 50.0);
    private_nh.param("minX", minX, -50.0);
    private_nh.param("minY", minY, -50.0);

    private_nh.param("deltaT", deltaT, 0.1);
    private_nh.param("deltaX", deltaX, 0.1);

    private_nh.param("greedy", greedy, 0.5);


    //TODO move elsewhere
    private_nh.param("maxU1", maxU1, 5.0);
    private_nh.param("maxU2", maxU2, 1.0);
    private_nh.param("minU1", minU1, -5.0);
    private_nh.param("minU2", minU2, -1.0);
    private_nh.param("discretization", discretization, 3);


    //TODO select from parameter
    std::cerr << "creating kinematic model" << std::endl;
    kinematicModel = new DifferentialDrive(*map);
    distance = new L2Distance();

    vis_pub = private_nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan)
{
    Distance& distance = *this->distance;

    VectorXd&& x0 = convertPose(start);
    VectorXd&& xGoal = convertPose(goal);

    RRT rrt(distance, x0);

    ROS_INFO("Planner started");

    for(unsigned int i = 0; i < K; i++)
    {

        VectorXd xRand;

        if(RandomGenerator::sampleEvent(greedy))
        {
        	xRand = xGoal;
        	//cerr << "sampled Goal state " << xRand.transpose() << endl;
        }
        else
        {
        	xRand = randomState();
        	//cerr << "sampled Random state " << xRand.transpose() << endl;
        }


        auto* node = rrt.searchNearestNode(xRand);

        //cerr << "Found NN " << node->x.transpose() << endl;

        VectorXd xNew;

        if(newState(xRand, node->x, xNew))
        {

        	//cerr << "Computed new state " << xNew.transpose() << endl;

            rrt.addNode(node, xNew);

            publishSegment(node->x, xNew);

            //cerr << "goal distance " << distance(xNew, xGoal) << endl;

            if(distance(xNew, xGoal) < deltaX)
            {
            	ROS_INFO("Plan found");
                return true;
            }
        }

        //cerr << "point " << i << endl;

    }

    //cerr << "Failed to found a plan"<< endl;
    return false;

}

VectorXd RRTPlanner::randomState()
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

bool RRTPlanner::newState(const VectorXd& xRand,
                          const VectorXd& xNear,
                          VectorXd& xNew)
{
    VectorXd uc(2);
    VectorXd x;

    double deltaU1 = (maxU1 - minU1) / (discretization-1.0);
    double deltaU2 = (maxU2 - minU2) / (discretization-1.0);

    double minDistance = std::numeric_limits<double>::infinity();
    Distance& distance = *this->distance;

    uc(0) = minU1;

    for(unsigned int i = 0; i < discretization; i++)
    {
        uc(1) = minU2;

        for(unsigned int j = 0; j < discretization; j++)
        {
        	try
        	{
				x = kinematicModel->compute(xNear, uc, deltaT);
				double currentDist = distance(xRand, x);

				if(currentDist < minDistance)
				{
					xNew = x;
					minDistance = currentDist;
				}
        	}
        	catch(...) //TODO catch specific exception
        	{

        	}


            uc(1) += deltaU2;
        }

        uc(0) += deltaU1;

    }

    return minDistance < std::numeric_limits<double>::infinity();
}

VectorXd RRTPlanner::convertPose(const geometry_msgs::PoseStamped& msg)
{
    auto& q_ros = msg.pose.orientation;
    auto& t_ros = msg.pose.position;

    Quaterniond q(q_ros.w, q_ros.x, q_ros.y, q_ros.z);

    Vector3d theta = q.matrix().eulerAngles(0, 1, 2);

    VectorXd x(3);
    x << t_ros.x, t_ros.y, theta(2);

    return x;
}

void RRTPlanner::publishSegment(const Eigen::VectorXd& xStart, const Eigen::VectorXd& xEnd)
{
	static int id = 0;

	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "rrt";
	marker.id = id++;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0;
	marker.scale.z = 0;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	geometry_msgs::Point p1;
	geometry_msgs::Point p2;

	p1.x = xStart(0);
	p1.y = xStart(1);
	p1.z = xStart(2);

	p2.x = xEnd(0);
	p2.y = xEnd(1);
	p2.z = xEnd(2);

	marker.points.push_back(p1);
	marker.points.push_back(p2);

	vis_pub.publish(marker);

}


};

