#include <ros/ros.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32MultiArray.h>


std::string odom_frame;
std::string base_link_frame;
int publish_rate;
double pixel_to_meter;
double separation;

class TfHandle
{
private:
    ros::Subscriber sub_driver;
    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform transform_odom2base_link;
    ros::Rate r;
    double lx, ly, rx, ry, yaw;

    void calculate_transform(const std_msgs::Int32MultiArray::ConstPtr& msg_in)
    {
        this->lx += double(msg_in->data[0])/pixel_to_meter;
        this->ly += double(msg_in->data[1])/pixel_to_meter;
        this->rx += double(msg_in->data[2])/pixel_to_meter;
        this->ry += double(msg_in->data[3])/pixel_to_meter;

        //ROS_INFO("%f\t%f\t%f\t%f", left_dx, left_dy, rght_dx, rght_dy);
        double cos_yaw = (this->lx - this->rx)/sqrt(pow(this->lx - this->rx, 2) + pow(this->ly - this->ry, 2));
        if (cos_yaw >  1) cos_yaw =  1.0;
        if (cos_yaw < -1) cos_yaw = -1.0;

        this->yaw = acos(cos_yaw);

        this->transform_odom2base_link.setOrigin(tf::Vector3((this->lx + this->rx)/2.0, (this->ly + this->ry)/2.0, 0.0) );
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
        this->lx = separation/2.0;
        this->ly = 0.0;
        this->rx = -separation/2.0;
        this->ry = 0.0;
        this->yaw = 0.0;

        this->sub_driver = nh.subscribe<std_msgs::Int32MultiArray> ("nucobot_mice_driver/xy_shift", 1, &TfHandle::calculate_transform, this);
        while (ros::ok()) {
            this->spin();
        }
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "nucobot_odometry_driver");
    ros::NodeHandle node;

    if (!node.getParam("odometry_driver/odom_frame", odom_frame)) odom_frame = "/odom";
    if (!node.getParam("odometry_driver/base_link_frame", base_link_frame)) base_link_frame = "/base_footprint";
    if (!node.getParam("odometry_driver/publish_rate", publish_rate)) publish_rate = 50;
    if (!node.getParam("odometry_driver/pixel_to_meter", pixel_to_meter)) pixel_to_meter = 1000; // 1 meter = pixel_to_meter pixels
    if (!node.getParam("odometry_driver/separation", separation)) separation = 0.10; // separation of sensors in meters

    TfHandle tf_handle(node);

    ros::spin();
    return 0;
};
