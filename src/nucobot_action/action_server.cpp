/*
 * action_server.cpp
 *
 *  Created on: Feb 16, 2015
 *      Author: vsevolod
 */

#include "action_server.h"

////////////////////////////////////////////////////////////////////////////////
//  ActionServer functions
////////////////////////////////////////////////////////////////////////////////

ActionServer::ActionServer(ros::NodeHandle nh_):
    as_achieve_target (nh_, "AchieveTargetAS", boost::bind(&ActionServer::achieveTargetCB, this, _1), false)
{
    this->position.x = NAN;
    this->position.y = NAN;
    this->target.x = NAN;
    this->target.y = NAN;
    this->target_name = "";

    as_achieve_target.start();
}

void ActionServer::achieveTargetCB(const nucobot_action::AchieveTargetGoalConstPtr  &goal)
{
    nucobot_action::AchieveTargetResult result_;
    ros::Rate r(60);

    this->set_closest_as_target("cylinder");

    ROS_INFO("%s\n", this->get_target_name().c_str());

    result_.success = true;
    as_achieve_target.setSucceeded(result_);
    return;
}

////////////////////////////////////////////////////////////////////////////////
//  Motion functions
////////////////////////////////////////////////////////////////////////////////

bool ActionServer::set_obj_map (gazebo_msgs::ModelStates obj_map_)
{
    this->obj_map = obj_map_;
    return true;
}

bool ActionServer::set_target (geometry_msgs::Pose2D target_)
{
    this->target.x     = target_.x;
    this->target.y     = target_.y;
    this->target.theta = target_.theta;
    return true;
}

geometry_msgs::Pose2D ActionServer::get_target()
{
    return this->target;
}

bool ActionServer::set_target_name (std::string name)
{
    this->target_name = name;
    return true;
}

std::string ActionServer::get_target_name()
{
    return this->target_name;
}

bool ActionServer::set_position(geometry_msgs::Pose2D position_)
{
    this->position = position_;
    return true;
}

geometry_msgs::Pose2D ActionServer::get_position()
{
    return this->position;
}

bool ActionServer::set_closest_as_target (std::string obj_name) {
    bool initialized = false;
    double min_dist = NAN;
    double tmp_dist = NAN;
    int target_num = -1;

    ROS_INFO("Odom: %lg | %lg", this->position.x, this->position.y);

    for (int i = 0; i < this->obj_map.name.size(); ++i) {
        if (this->obj_map.name[i].find(obj_name) != std::string::npos) {
            tmp_dist = distance (this->position.x, this->position.y,
                                 this->obj_map.pose[i].position.x, this->obj_map.pose[i].position.y);

            ROS_INFO("Obj name: %s\n | %lg | %lg", this->obj_map.name[i].c_str(), tmp_dist, min_dist);

            if (!initialized) {
                min_dist = tmp_dist;
                initialized = true;
                target_num = i;
            }

            if (initialized && tmp_dist < min_dist) {
                min_dist = tmp_dist;
                target_num = i;
            }
        }
    }

    ROS_INFO("Target name: %s\n", this->get_target_name().c_str());

    if (!isnan(min_dist) && target_num >= 0) {
        this->target_name = this->obj_map.name[target_num];
        this->target.x = this->obj_map.pose[target_num].position.x;
        this->target.y = this->obj_map.pose[target_num].position.y;
        return true;
    }

    return false;
}


////////////////////////////////////////////////////////////////////////////////
//  Utility functions
////////////////////////////////////////////////////////////////////////////////

double distance (double x1, double y1, double x2, double y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

