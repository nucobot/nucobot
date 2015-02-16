/*
 * action_server.cpp
 *
 *  Created on: Feb 14, 2015
 *      Author: Vsevolod Livinskiy
 */

#include <ros/ros.h>

// base class for action server
#include <actionlib/server/simple_action_server.h>

// Action description files (check the /action dir)
#include <nucobot_action/AchieveTargetAction.h>

class ActionServer
{
public:
    ActionServer(ros::NodeHandle nh_) :
        as_achieve_target (nh_, "AchieveTargetAS", boost::bind(&ActionServer::achieveTargetCB, this, _1), false)
    {
        as_achieve_target.start();
    }

    ~ActionServer(void) {}

    void achieveTargetCB(const nucobot_action::AchieveTargetGoalConstPtr  &goal) {
        nucobot_action::AchieveTargetResult result_;
        ros::Rate r(60);

        while(true) {
            ROS_INFO("State is ALIVE!!!!!!");
            r.sleep();
        }
        result_.success = true;
        as_achieve_target.setSucceeded(result_);
        return;
    }

private:
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<nucobot_action::AchieveTargetAction>   as_achieve_target;

};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "nucobot_action_server");
    ros::NodeHandle nh;


    ActionServer action_server(nh);


    ros::spin ();

    return 0;
};


