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


#include <ros/ros.h>
#include <Eigen/Dense>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace Eigen;

Affine3d T;

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	T.setIdentity();
	T.translation()(0) = msg->pose.pose.position.x;
	T.translation()(1) = msg->pose.pose.position.y;
	T.translation()(2) = msg->pose.pose.position.z;

	auto q_ros = msg->pose.pose.orientation;
	Quaterniond q(q_ros.w, q_ros.x, q_ros.y, q_ros.z);
	T.rotate(q);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_node");

	ros::NodeHandle n;

	if(argc < 4)
	{
		ROS_ERROR("You must specify parent farme, child frame and publisher frequency");

		return -1;
	}


	tf2_ros::TransformBroadcaster tfBroadcaster;


	std::string parentFrame = argv[1];
	std::string childFrame = argv[2];
	double frequency = std::stod(argv[3]);


	T.setIdentity();

	ros::Subscriber sub = n.subscribe("/initialpose", 10, initialPoseCallback);
	//n.subscribe("/cmd_vel", 1, velocityCallback);

	ros::Rate rate(frequency);

	while(ros::ok())
	{
		auto tf_msg = tf2::eigenToTransform(T);
		tf_msg.header.stamp = ros::Time::now();
		tf_msg.header.frame_id = parentFrame;
		tf_msg.child_frame_id = childFrame;

		tfBroadcaster.sendTransform(tf_msg);
		ros::spinOnce();
		rate.sleep();
	}


}
