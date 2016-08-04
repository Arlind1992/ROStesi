/*
 * RTTPlanner.h
 *
 *  Created on: 28 lug 2016
 *      Author: dave
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


namespace rtt_planning
{

class RTTPlanner : public nav_core::BaseGlobalPlanner
{
public:

    RTTPlanner();
    RTTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    costmap_2d::Costmap2DROS* costmap;
};
};



#endif /* INCLUDE_RTTPLANNER_H_ */
