#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"


/**
 * imu_driver_node - convertor imu data from arduino in geometry_msgs::poseStamp format to sensor_msgs::Imu
 * Input topic - imu_raw_data
 * Output topic - imu_data 
 */

ros::Publisher converter_pub;
ros::Subscriber converter_sub;
sensor_msgs::Imu output_data; 

void converterCallback(const geometry_msgs::PoseStamped::ConstPtr& input_data) 
{
    output_data.angular_velocity.x = input_data->pose.position.x;
    output_data.angular_velocity.y = input_data->pose.position.y;
    output_data.angular_velocity.z = input_data->pose.position.z;
    output_data.linear_acceleration.x = input_data->pose.orientation.x;
    output_data.linear_acceleration.y = input_data->pose.orientation.y;
    output_data.linear_acceleration.z = input_data->pose.orientation.z;
    output_data.header = input_data->header;
    converter_pub.publish(output_data);
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "imu_data");
    ros::NodeHandle n;
    converter_pub = n.advertise<sensor_msgs::Imu>("imu_data", 1);
    converter_sub = n.subscribe("imu_raw_data", 1, converterCallback);

    while (ros::ok()) {
      ros::spin();
    }
    return 0;
}
