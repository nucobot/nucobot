/*
 * action_server.cpp
 *
 *  Created on: Feb 14, 2015
 *      Author: Vsevolod Livinskiy
 */

#include "action_server.h"

ActionServer *act_srv;

ros::Subscriber sub_obj;
ros::Subscriber sub_odom;
ros::Publisher  pub_target_obj;

void obj_map_callback(const gazebo_msgs::ModelStates::ConstPtr &data)
{
    act_srv->set_obj_map(*data);
    std_msgs::String target_obj_msg;
    target_obj_msg.data = act_srv->get_target_name().c_str();
    pub_target_obj.publish(target_obj_msg);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    geometry_msgs::Pose2D position_;
    position_.x = odom->pose.pose.position.x;
    position_.y = odom->pose.pose.position.y;
    act_srv->set_position(position_);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "nucobot_action_server");
    ros::NodeHandle nh;


    ActionServer action_server(nh);
    act_srv = &action_server;

    sub_obj        = nh.subscribe<gazebo_msgs::ModelStates> ("fake_towermap/objects", 1, obj_map_callback);
    sub_odom       = nh.subscribe<nav_msgs::Odometry> ("odom", 1, odom_callback);
    pub_target_obj = nh.advertise<std_msgs::String> ("action_server/target_objects", 1);

    ros::spin ();
    return 0;
};


