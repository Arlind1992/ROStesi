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

    deltaT = 0;
    deltaX = 0;

    //TODO move elsewhere
    maxU1 = 0;
    maxU2 = 0;
    minU1 = 0;
    minU2 = 0;
    discretization = 0;

    kinematicModel = nullptr;
    distance = nullptr;
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

    private_nh.param("deltaT", deltaT, 0.5);
    private_nh.param("deltaX", deltaX, 0.1);


    //TODO move elsewhere
    private_nh.param("maxU1", maxU1, 5.0);
    private_nh.param("maxU2", maxU2, 1.0);
    private_nh.param("minU1", minU1, -5.0);
    private_nh.param("minU2", minU2, -1.0);
    private_nh.param("discretization", discretization, 3);


    //TODO select from parameter
    kinematicModel = new DifferentialDrive();
    distance = new L2Distance();
}

bool RTTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan)
{
    Distance& distance = *this->distance;

    VectorXd&& x0 = convertPose(start);
    VectorXd&& xGoal = convertPose(goal);

    RTT rtt(distance, x0);

    ROS_INFO("Planner started");

    for(unsigned int i = 0; i < K; i++)
    {
        auto&& xRand = randomState();

        cerr << "sampled Random state " << xRand.transpose() << endl;
        auto* node = rtt.searchNearestNode(xRand);

        cerr << "Found NN " << node->x.transpose() << endl;

        VectorXd xNew;

        if(newState(xRand, node->x, xNew))
        {

        	cerr << "Computed new state " << xNew.transpose() << endl;

            rtt.addNode(node, xNew);

            cerr << "goal distance " << distance(xNew, xGoal) << endl;

            if(distance(xNew, xGoal) < deltaX)
            {
                cout << "Plan found" << endl;
                return true;
            }
        }

        cerr << "point " << i << endl;

    }

    cerr << "Failed to found a plan"<< endl;
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

bool RTTPlanner::newState(const VectorXd& xRand,
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
            x = kinematicModel->compute(xNear, uc, deltaT);
            double currentDist = distance(xRand, x);

            if(currentDist < minDistance)
            {
                xNew = x;
                minDistance = currentDist;
            }


            uc(1) += deltaU2;
        }

        uc(0) += deltaU1;

    }

    return true;
}

VectorXd RTTPlanner::convertPose(const geometry_msgs::PoseStamped& msg)
{
    auto& q_ros = msg.pose.orientation;
    auto& t_ros = msg.pose.position;

    Quaterniond q(q_ros.w, q_ros.x, q_ros.y, q_ros.z);

    Vector3d theta = q.matrix().eulerAngles(0, 1, 2);

    VectorXd x(3);
    x << t_ros.x, t_ros.y, theta(2);

    return x;
}


};
