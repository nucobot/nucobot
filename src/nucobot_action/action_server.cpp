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
    move_base_ac("move_base", true),
    as_achieve_target (nh_, "AchieveTargetAS", boost::bind(&ActionServer::achieveTargetCB, this, _1), false)
{
    //Waiting for MOveBase action server to start.
    ros::Duration timeout(3);
    move_base_ac.waitForServer(timeout);

    this->position.pose.position.x = NAN;
    this->position.pose.position.y = NAN;
    this->position.pose.position.z = NAN;
    this->position.pose.orientation.x = NAN;
    this->position.pose.orientation.y = NAN;
    this->position.pose.orientation.z = NAN;
    this->position.pose.orientation.w = NAN;
    this->target.x = NAN;
    this->target.y = NAN;
    this->target_name = "";

    as_achieve_target.start();
}

void ActionServer::achieveTargetCB(const nucobot_action::AchieveTargetGoalConstPtr  &goal)
{
    nucobot_action::AchieveTargetResult result_;
    ros::Rate r(60);

    //this->set_closest_as_target("cup");

    std::string targ_name;

    do {
        std::cin >> targ_name;
    } while (!this->set_target_name(targ_name));

    ROS_ERROR("%s: %lg | %lg", this->target_name.c_str(), this->target.x, this->target.y);


    move_base_msgs::MoveBaseGoal move_base_goal;
    move_base_goal.target_pose.pose.position.x = this->target.x;
    move_base_goal.target_pose.pose.position.y = this->target.y;
    move_base_goal.target_pose.pose.orientation = this->position.pose.orientation;
    move_base_goal.target_pose.header.frame_id = "map";
    this->move_base_ac.sendGoal(move_base_goal);

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
    return true;
}

geometry_msgs::Pose2D ActionServer::get_target()
{
    return this->target;
}

bool ActionServer::set_target_name (std::string name)
{
    this->target_name = name;
    for (int i = 0; i < this->obj_map.name.size(); ++i) {
        if (this->obj_map.name[i] == name) {
            this->target.x = this->obj_map.pose[i].position.x;
            this->target.y = this->obj_map.pose[i].position.y;
            return true;
        }
    }
    return false;
}

std::string ActionServer::get_target_name()
{
    return this->target_name;
}

bool ActionServer::set_position(geometry_msgs::PoseStamped position_)
{
    this->position = position_;
    return true;
}

geometry_msgs::PoseStamped ActionServer::get_position()
{
    return this->position;
}

bool ActionServer::set_closest_as_target (std::string obj_name) {
    bool initialized = false;
    double min_dist = NAN;
    double tmp_dist = NAN;
    int target_num = -1;

    for (int i = 0; i < this->obj_map.name.size(); ++i) {
        if (this->obj_map.name[i].find(obj_name) != std::string::npos) {
            tmp_dist = distance (this->position.pose.position.x, this->position.pose.position.y,
                                 this->obj_map.pose[i].position.x, this->obj_map.pose[i].position.y);

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

