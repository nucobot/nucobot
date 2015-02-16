/*
 * action_server.h
 *
 *  Created on: Feb 16, 2015
 *      Author: vsevolod
 */

#include <cmath>
#include <climits>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

// base class for action server
#include <actionlib/server/simple_action_server.h>

// Action description files (check the /action dir)
#include <nucobot_action/AchieveTargetAction.h>

#ifndef ACTION_SERVER_H_
#define ACTION_SERVER_H_

////////////////////////////////////////////////////////////////////////////////
//  ActionServer
////////////////////////////////////////////////////////////////////////////////

class ActionServer
{
private:
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<nucobot_action::AchieveTargetAction>   as_achieve_target;

    // Motion variables
    geometry_msgs::Pose2D position;
    geometry_msgs::Pose2D target;
    std::string target_name;
    gazebo_msgs::ModelStates obj_map;

public:
    ActionServer(ros::NodeHandle nh_);
    ~ActionServer(void) {}

    void achieveTargetCB(const nucobot_action::AchieveTargetGoalConstPtr  &goal);

    // Motion functions
    bool set_obj_map (gazebo_msgs::ModelStates obj_map_);
    bool set_target (geometry_msgs::Pose2D target_);
    geometry_msgs::Pose2D get_target();
    bool set_target_name(std::string name);
    std::string get_target_name();
    bool set_position(geometry_msgs::Pose2D position_);
    geometry_msgs::Pose2D get_position();
    bool set_closest_as_target (std::string obj_name);
};

////////////////////////////////////////////////////////////////////////////////
//  Utility functions
////////////////////////////////////////////////////////////////////////////////

double distance (double x1, double y1, double x2, double y2);

#endif /* ACTION_SERVER_H_ */
