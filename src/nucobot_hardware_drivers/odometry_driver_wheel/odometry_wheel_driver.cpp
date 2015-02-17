#include <ros/ros.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>


std::string odom_frame;
std::string base_link_frame;
int publish_rate;
double raw_to_meter;
double separation;

class TfHandle
{
private:
    ros::Subscriber sub_driver;
    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform transform_odom2base_link;
    ros::Rate r;
    double x, y, yaw;
    double t;

    void calculate_transform(const geometry_msgs::Point::ConstPtr& msg_in)
    {
    	double dt = ros::Time::now().toSec() - this->t;
        this->t = ros::Time::now().toSec();

        double lx = dt*double(msg_in->x)/raw_to_meter;
        double rx = dt*double(msg_in->y)/raw_to_meter;

        double dyaw = (lx - rx)/separation;
        double dx = rx, dy = 0;
        if (dyaw != 0) {
        	dx = sin(dyaw) * (separation/2 + rx/dyaw);
        	dy = (1 - cos(dyaw)) * (separation/2 + rx/dyaw);
        }

        this->x += dx * cos(this->yaw) - dy * sin(this->yaw);
        this->y += dx * sin(this->yaw) + dy * cos(this->yaw);
        this->yaw += dyaw;

        this->transform_odom2base_link.setOrigin(tf::Vector3(this->x, this->y, 0.0) );
        this->transform_odom2base_link.setRotation(tf::createQuaternionFromRPY(0, 0, yaw));
    }

    void spin()
    {
        ros::spinOnce();
        this->tf_broadcaster.sendTransform(tf::StampedTransform(transform_odom2base_link, ros::Time::now(),
        		                                                    odom_frame, base_link_frame));
        this->r.sleep();
    }

public:
    TfHandle(ros::NodeHandle nh): r(publish_rate)
    {
        this->transform_odom2base_link.setIdentity();
        this->x = 0.0;
        this->y = 0.0;
        this->yaw = 0.0;
        this->t = ros::Time::now().toSec();

        this->sub_driver = nh.subscribe<geometry_msgs::Point> ("raw_wheel_odometry", 1, &TfHandle::calculate_transform, this);
        while (ros::ok()) {
            this->spin();
        }
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "nucobot_wheel_odometry_driver");
    ros::NodeHandle node;

    if (!node.getParam("wheel_odometry_driver/odom_frame", odom_frame)) odom_frame = "/odom";
    if (!node.getParam("wheel_odometry_driver/base_link_frame", base_link_frame)) base_link_frame = "/base_footprint";
    if (!node.getParam("wheel_odometry_driver/publish_rate", publish_rate)) publish_rate = 50;
    if (!node.getParam("wheel_odometry_driver/raw_to_meter", raw_to_meter)) raw_to_meter = 100; // 1 meter per second = raw_to_meter of raw data
    if (!node.getParam("wheel_odometry_driver/separation", separation)) separation = 0.10; // separation of sensors in meters

    TfHandle tf_handle(node);

    ros::spin();
    return 0;
};
