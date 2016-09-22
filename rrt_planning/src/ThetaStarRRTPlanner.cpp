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

#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

#include "rrt_planning/ThetaStarRRTPlanner.h"
#include "rrt_planning/map/ROSMap.h"
#include "rrt_planning/kinematics_models/DifferentialDrive.h"
#include "rrt_planning/local_planner/MotionPrimitivesPlanner.h"
#include "rrt_planning/utils/RandomGenerator.h"
#include "rrt_planning/rrt/RRT.h"

using namespace Eigen;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planning::ThetaStarRRTPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rrt_planning
{

ThetaStarRRTPlanner::ThetaStarRRTPlanner()
{
    K = 0;
    deltaX = 0;
    laneWidth = 0;
	greedy = 0;

    map = nullptr;
    kinematicModel = nullptr;
    distance = nullptr;
    localPlanner = nullptr;

    thetaStarPlanner = new ThetaStarPlanner();
}

ThetaStarRRTPlanner::ThetaStarRRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
	thetaStarPlanner = new ThetaStarPlanner(name, costmap_ros);
}


void ThetaStarRRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	thetaStarPlanner->initialize(name, costmap_ros);

    map = new ROSMap(costmap_ros);

    //Get parameters from ros parameter server
    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("iterations", K, 30000);
    private_nh.param("deltaX", deltaX, 0.5);
	private_nh.param("laneWidth", laneWidth, 2.0);
	private_nh.param("greedy", greedy, 0.1);
	private_nh.param("deltaTheta", deltaTheta, M_PI/4);

    // create kinematic, distance and local planner //TODO select from parameters
    kinematicModel = new DifferentialDrive();
    distance = new L2ThetaDistance();
    localPlanner = new MotionPrimitivesPlanner(*map, *distance, *kinematicModel);


    localPlanner->initialize(private_nh);

    // Advertise the visualizer
    vis_pub = private_nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
}

bool ThetaStarRRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan)
{
	cleanSegments();

	// Retrive Theta* plan
	vector<geometry_msgs::PoseStamped> thetaStarPlan;

	if(!thetaStarPlanner->makePlan(start, goal, thetaStarPlan)) 
	{
		ROS_INFO("Impossible to compute the Theta* plan");
		return false;
	}

	publishThetaStar(thetaStarPlan);

	// Compute RRT-Theta* plan
    Distance& distance = *this->distance;

    VectorXd&& x0 = convertPose(start);
    VectorXd&& xGoal = convertPose(goal);

    RRT rrt(distance, x0);

    ROS_INFO("Theta*-RRT started");

    for(unsigned int i = 0; i < K; i++)
    {

        VectorXd xRand;

		if(RandomGenerator::sampleEvent(greedy))
			xRand = xGoal;
		else
			xRand = kinematicModel->anyAngleSampling(thetaStarPlan, laneWidth, deltaTheta);

		addPoint(xRand);

        auto* node = rrt.searchNearestNode(xRand);

        VectorXd xNew;

        if(newState(xRand, node->x, xNew))
        {
            rrt.addNode(node, xNew);

            publishSegment(node->x, xNew);

            if(distance(xNew, xGoal) < deltaX)
            {
                auto&& path = rrt.getPathToLastNode();
                publishPlan(path, plan, start.header.stamp);

                ROS_INFO("Plan found");

                return true;
            }
        }

    }

    ROS_WARN_STREAM("Failed to found a plan in " << K << " RRT iterations");
    return false;

}


bool ThetaStarRRTPlanner::newState(const VectorXd& xRand,
                          const VectorXd& xNear,
                          VectorXd& xNew)
{
    return localPlanner->compute(xNear, xRand, xNew);
}

VectorXd ThetaStarRRTPlanner::convertPose(const geometry_msgs::PoseStamped& msg)
{
    auto& q_ros = msg.pose.orientation;
    auto& t_ros = msg.pose.position;

    Quaterniond q(q_ros.w, q_ros.x, q_ros.y, q_ros.z);

    Vector3d theta = q.matrix().eulerAngles(0, 1, 2);

    VectorXd x(3);
    x << t_ros.x, t_ros.y, theta(2);

    return x;
}

void ThetaStarRRTPlanner::publishPlan(std::vector<VectorXd>& path,
                             std::vector<geometry_msgs::PoseStamped>& plan, const ros::Time& stamp)
{
    for(auto x : path)
    {
        geometry_msgs::PoseStamped msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = "map";

        msg.pose.position.x = x(0);
        msg.pose.position.y = x(1);
        msg.pose.position.z = 0;

        Matrix3d m;
        m = AngleAxisd(x(2), Vector3d::UnitZ())
            * AngleAxisd(0, Vector3d::UnitY())
            * AngleAxisd(0, Vector3d::UnitX());

        Quaterniond q(m);

        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        plan.push_back(msg);
    }
}

void ThetaStarRRTPlanner::cleanSegments()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "rrt";
    marker.action = visualization_msgs::Marker::DELETEALL;

    vis_pub.publish(marker);

	marker.ns = "thetastar";

	vis_pub.publish(marker);

	marker.ns = "point";

	vis_pub.publish(marker);
}

void ThetaStarRRTPlanner::publishSegment(const VectorXd& xStart, const VectorXd& xEnd)
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
    marker.scale.x = 0.05;
    marker.scale.y = 0;
    marker.scale.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

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

    //ros::Duration(0.1).sleep();
}


void ThetaStarRRTPlanner::addPoint(VectorXd& xStart)
{
    static int id = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "point";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    geometry_msgs::Point p1;

    p1.x = xStart(0);
    p1.y = xStart(1);
    p1.z = 0;

    marker.points.push_back(p1);

    vis_pub.publish(marker);

    //ros::Duration(0.1).sleep();
}


void ThetaStarRRTPlanner::publishThetaStar(std::vector<geometry_msgs::PoseStamped>& plan)
{
    static int id = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "thetastar";
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
    marker.scale.x = 0.05;
    marker.scale.y = 0;
    marker.scale.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

	for(int i = 0; i < plan.size(); i++)
	{
		geometry_msgs::Point p;

		p.x = plan[i].pose.position.x;
		p.y = plan[i].pose.position.y;
		p.z = plan[i].pose.position.z;

	    marker.points.push_back(p);
	}

    vis_pub.publish(marker);

    //ros::Duration(0.1).sleep();
}


ThetaStarRRTPlanner::~ThetaStarRRTPlanner()
{
	if(kinematicModel)
		delete kinematicModel;

	if(distance)
		delete distance;

	if(localPlanner)
		delete localPlanner;

	if(map)
		delete map;

	if(thetaStarPlanner)
		delete thetaStarPlanner;

}


};

