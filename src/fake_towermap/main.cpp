#include <vector>
#include <cmath>
#include <cstring>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point32.h>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


ros::Subscriber sub_gazbo;
ros::Publisher  pub_markr;
ros::Publisher  pub_cloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl (new pcl::PointCloud<pcl::PointXYZ>);

void callback(const gazebo_msgs::ModelStates::ConstPtr &data)
{
	visualization_msgs::MarkerArray ma;
    pcl->header.frame_id = "world";
    pcl->header.stamp = ros::Time();
    //pcl->points = static_map_borders // this is obviously a cludge, the static map should be generated elsewhere

    for (int i = 0; i < data->name.size(); ++i) {
	    visualization_msgs::Marker marker, text_marker;
        marker.header.frame_id = "world";
        marker.id = 0;
        marker.ns = data->name[i];
        marker.header.stamp = ros::Time();
        marker.lifetime = ros::Time(0.3);
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = data->pose[i];

        text_marker.header.frame_id = "world";
        text_marker.id = 1;
        text_marker.ns = data->name[i];
        text_marker.header.stamp = ros::Time();
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.lifetime = ros::Time(0.3);
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose = data->pose[i];
        text_marker.text = data->name[i];
        text_marker.scale.x = 0.10;
        text_marker.scale.y = 0.10;
        text_marker.scale.z = 0.05;
        text_marker.color.a = 0.90;
        text_marker.color.r = 1.00;
        text_marker.color.g = 1.00;
        text_marker.color.b = 1.00;

        if (data->name[i].find("cylinder") != std::string::npos) {
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.scale.x = 0.060;
            marker.scale.y = 0.060;
            marker.scale.z = 0.070;
            marker.color.a = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            ma.markers.push_back(marker);
            ma.markers.push_back(text_marker);
        }

        if (data->name[i].find("tennis") != std::string::npos) {
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.064;
            marker.scale.y = 0.064;
            marker.scale.z = 0.064;
            marker.color.a = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            ma.markers.push_back(marker);
            ma.markers.push_back(text_marker);
        }

        if (data->name[i].find("pop_corn") != std::string::npos) {
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.040;
            marker.scale.y = 0.040;
            marker.scale.z = 0.040;
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            ma.markers.push_back(marker);
            ma.markers.push_back(text_marker);
        }

        if (data->name[i].find("cup") != std::string::npos) {
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.pose.position.z += 0.075;
            marker.scale.x = 0.095;
            marker.scale.y = 0.095;
            marker.scale.z = 0.150;
            marker.color.a = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            ma.markers.push_back(marker);
            ma.markers.push_back(text_marker);
            //pcl.points += gen_flatcircle(marker.scale.x / 2.0, marker.pose.position.x, marker.pose.position.y)
        }

    //pub_cloud.publish(pcl);
	pub_markr.publish(ma);
};



int main( int argc, char** argv)
{
    ros::init(argc, argv, "fake_towermap");
    ros::NodeHandle nh;

    sub_gazbo = nh.subscribe<gazebo_msgs::ModelStates>  ("/gazebo/model_states", 1, callback);
    pub_markr = nh.advertise<visualization_msgs::MarkerArray> ("visualization/fake_towermap/objects", 1);
	pub_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("fake_towermap/pointcloud", 1);

    ros::spin ();
    return 0;
};
