#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <wheel_driver/WheelState.h>
#include <wheel_driver/WheelBaseState.h>


class WheelDriver {
public:
    ros::NodeHandle nh;
    ros::Publisher wheel_state_pub;
    ros::Subscriber wheel_state_sub;
    ros::Subscriber vel_sub;
    geometry_msgs::Pose2D vel_curr;
    wheel_driver::WheelBaseState wheel_state_curr;
    std::string wheel_state_real_topic;
    std::string wheel_state_topic;
    std::string vel_real_topic;
    bool is_simulated;

public:
    WheelDriver ();
    void wheelStateCallback(const wheel_driver::WheelBaseState& msg);
    void velCallback(const geometry_msgs::Pose2D& msg);

};

WheelDriver::WheelDriver()
{

    // TODO: replace it with reading from param.yaml file
    /*
    if (!this->nh.getParam("wheel_driver/wheel_state_real_topic", this->wheel_state_real_topic))
            ROS_ERROR("Failed to get param 'wheel_state_real_topic'");
    if (!this->nh.getParam("wheel_driver/wheel_state_topic", this->wheel_state_topic))
            ROS_ERROR("Failed to get param 'wheel_state_topic'");
    if (!this->nh.getParam("wheel_driver/vel_real_topic", this->vel_real_topic))
            ROS_ERROR("Failed to get param 'vel_real_topic'");
    if (!this->nh.getParam("wheel_driver/is_simulated", this->is_simulated))
            ROS_ERROR("Failed to get param 'is_simulated'");
    */

    this->wheel_state_real_topic = "wheel_state";
    this->wheel_state_topic      = "wheel_state";
    this->vel_real_topic         = "vel_real";
    this->is_simulated           = true;

    this->vel_curr.x = 0;
    this->vel_curr.y = 0;
    this->vel_curr.theta = 0;

    this->wheel_state_curr.wheel_0.vel = 0;
    this->wheel_state_curr.wheel_0.rot = 0;
    this->wheel_state_curr.wheel_1.vel = 0;
    this->wheel_state_curr.wheel_1.rot = 0;
    this->wheel_state_curr.wheel_2.vel = 0;
    this->wheel_state_curr.wheel_2.rot = 0;
    this->wheel_state_curr.wheel_3.vel = 0;
    this->wheel_state_curr.wheel_3.rot = 0;
}

void WheelDriver::wheelStateCallback(const wheel_driver::WheelBaseState& msg)
{
    this->wheel_state_curr.wheel_0.vel = msg.wheel_0.vel;
    this->wheel_state_curr.wheel_0.rot = msg.wheel_0.rot;
    this->wheel_state_curr.wheel_1.vel = msg.wheel_1.vel;
    this->wheel_state_curr.wheel_1.rot = msg.wheel_1.rot;
    this->wheel_state_curr.wheel_2.vel = msg.wheel_2.vel;
    this->wheel_state_curr.wheel_2.rot = msg.wheel_2.rot;
    this->wheel_state_curr.wheel_3.vel = msg.wheel_3.vel;
    this->wheel_state_curr.wheel_3.rot = msg.wheel_3.rot;

    ROS_INFO("I heard: [%g]", this->wheel_state_curr.wheel_0.vel);
}

void WheelDriver::velCallback(const geometry_msgs::Pose2D& msg)
{
    this->vel_curr.x = msg.x;
    this->vel_curr.y = msg.y;
    this->vel_curr.theta = msg.theta;
    ROS_INFO("I heard: [%g]", this->vel_curr.x);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "wheel_driver");

  WheelDriver wheel_driver;

  wheel_driver.wheel_state_pub = wheel_driver.nh.advertise<wheel_driver::WheelBaseState>(wheel_driver.wheel_state_topic, 1000);
  wheel_driver.wheel_state_sub = wheel_driver.nh.subscribe(wheel_driver.wheel_state_real_topic,
                                      1000, &WheelDriver::wheelStateCallback, &wheel_driver);
  wheel_driver.vel_sub         = wheel_driver.nh.subscribe(wheel_driver.vel_real_topic,
                                      1000, &WheelDriver::velCallback, &wheel_driver);

  ros::Rate loop_rate(10);

  int i = 0;
  int count = 0;
  while (ros::ok())
  {

    wheel_driver::WheelBaseState msg;

    msg.wheel_0.vel = i++;

    ROS_INFO("%g", msg.wheel_0.vel);

    wheel_driver.wheel_state_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
