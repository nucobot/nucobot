#include <ros/ros.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32MultiArray.h>

typedef long long int int64;

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
    double x, y, yaw;
    int64 lx_last, rx_last;

    void calculate_transform(const std_msgs::Int32MultiArray::ConstPtr& msg_in)
    {
        double m_lx = double(msg_in->data[1] - this->lx_last) / pixel_to_meter;
        double m_rx = double(msg_in->data[3] - this->rx_last) / pixel_to_meter;
        this->lx_last = msg_in->data[1];
        this->rx_last = msg_in->data[3];

        double dyaw = (m_lx - m_rx)/separation;
    
        //ROS_INFO("%f\t%f\t%f\t%lld\t%d", m_lx, m_rx, dyaw, this->rx_last, msg_in->data[3]);
        double dx = m_rx, dy = 0;
        if (dyaw != 0) {
        	dx = sin(dyaw) * (separation/2 + m_rx/dyaw);
        	dy = (1 - cos(dyaw)) * (separation/2 + m_rx/dyaw);
        }

        this->x += dx * cos(this->yaw) - dy * sin(this->yaw);
        this->y += dx * sin(this->yaw) + dy * cos(this->yaw);
        this->yaw += dyaw;

        this->transform_odom2base_link.setOrigin(tf::Vector3(this->x, this->y, 0.0) );
        this->transform_odom2base_link.setRotation(tf::createQuaternionFromRPY(0, 0, this->yaw));
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
        this->lx_last = 0;
        this->rx_last = 0;
        this->sub_driver = nh.subscribe<std_msgs::Int32MultiArray> ("nucobot_mice_driver/xy_shift", 1, &TfHandle::calculate_transform, this);
        while (ros::ok()) {
            this->spin();
        }
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "nucobot_optical_odometry_driver");
    ros::NodeHandle node;

    if (!node.getParam("optical_odometry_driver/odom_frame", odom_frame)) odom_frame = "/odom";
    if (!node.getParam("optical_odometry_driver/base_link_frame", base_link_frame)) base_link_frame = "/base_footprint";
    if (!node.getParam("optical_odometry_driver/publish_rate", publish_rate)) publish_rate = 50;
    if (!node.getParam("optical_odometry_driver/pixel_to_meter", pixel_to_meter)) pixel_to_meter = 10000; // 1 meter = pixel_to_meter pixels
    if (!node.getParam("optical_odometry_driver/separation", separation)) separation = 0.10; // separation of sensors in meters

    TfHandle tf_handle(node);

    ros::spin();
    return 0;
};
