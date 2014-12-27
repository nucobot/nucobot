#!/usr/bin/env python

import re, math
import rospy, tf
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

pub = rospy.Publisher('visualization/fake_towermap/objects', MarkerArray, queue_size=10)
pub_pcl = rospy.Publisher('fake_towermap/pointcloud', PointCloud, queue_size=10)
# br = tf.TransformBroadcaster()




def gen_flatline(x1, y1, x2, y2):
    ret = []
    step = 0.04
    ln = math.sqrt((x2-x1)**2+(y2-y1)**2)
    dlx = step*(x2 - x1)/ln
    dly = step*(y2 - y1)/ln

    for i in xrange(int(ln/step)):
        ret.append(Point32(x1 + i*dlx, y1 + i*dly, 0))
    return ret

def gen_flatrect(x1, y1, x2, y2, step = 0.02):
    ret = []

    for i in xrange(int((x2 - x1)/step)):
        for j in xrange(int((y2 - y1)/step)):
            ret.append(Point32(x1 + i*step, y1 + j*step, 0))
    return ret

def gen_flatcircle(r, x, y, step = 0.5):
    ret = []
    phi = 0
    while (phi < 6.3):
        ret.append(Point32(x + r*math.sin(phi), y + r*math.cos(phi), 0))
        phi += step

    return ret

def genstaticmap():
    ret = []
    ret += gen_flatline(0, 0, 2, 0)
    ret += gen_flatline(2, 0, 2, 3)
    ret += gen_flatline(2, 3, 0, 3)
    ret += gen_flatline(0, 3, 0, 0)

    ret += gen_flatline(0.8, 0.0, 0.8, 0.4)
    ret += gen_flatline(0.8, 3.0, 0.8, 2.6)
    ret += gen_flatline(1.2, 0.0, 1.2, 0.4)
    ret += gen_flatline(1.2, 3.0, 1.2, 2.6)

    ret += gen_flatline(0, 0.97, 0.58, 0.97)
    ret += gen_flatline(0, 1.50, 0.58, 1.50)
    ret += gen_flatline(0, 2.03, 0.58, 2.03)

    ret += gen_flatrect(0, (300.0-31.0)/1000.0, 0.062, (300.0+31.0)/1000.0)
    ret += gen_flatrect(0, (600.0-31.0)/1000.0, 0.062, (600.0+31.0)/1000.0)
    ret += gen_flatrect(0, (2400.0-31.0)/1000.0, 0.062, (2400.0+31.0)/1000.0)
    ret += gen_flatrect(0, (2700.0-31.0)/1000.0, 0.062, (2700.0+31.0)/1000.0)
    return ret


static_map_borders = genstaticmap()




def callback(data):
    ma = MarkerArray()
    pcl = PointCloud()
    pcl.header.frame_id = "world"
    pcl.header.stamp = rospy.Time()
    pcl.points = static_map_borders # this is obviously a cludge, the static map should be generated elsewhere

    for i, name in enumerate(data.name):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.id = 0
        marker.ns = name  
        marker.header.stamp = rospy.Time()
        marker.lifetime = rospy.Time(0.3)
        marker.action = Marker.ADD
        marker.pose = data.pose[i]

        text_marker = Marker()
        text_marker.header.frame_id = "world"
        text_marker.id = 1
        text_marker.ns = name  
        text_marker.header.stamp = rospy.Time()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.lifetime = rospy.Time(0.3)
        text_marker.action = Marker.ADD
        text_marker.pose = data.pose[i]
        text_marker.text = name
        text_marker.scale.x = 0.1
        text_marker.scale.y = 0.1
        text_marker.scale.z = 0.05
        text_marker.color.a = 0.9
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0

        
        if 'cylinder' in name:
            marker.type = Marker.CYLINDER
            marker.scale.x = 0.060
            marker.scale.y = 0.060
            marker.scale.z = 0.070
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            ma.markers.append(marker)
            ma.markers.append(text_marker)
            pcl.points += gen_flatcircle(marker.scale.x / 2.0, marker.pose.position.x, marker.pose.position.y)
       
        if 'tennis' in name:
            marker.type = Marker.SPHERE
            marker.scale.x = 0.064
            marker.scale.y = 0.064
            marker.scale.z = 0.064
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            ma.markers.append(marker)
            ma.markers.append(text_marker)
            pcl.points += gen_flatcircle(marker.scale.x / 2.0, marker.pose.position.x, marker.pose.position.y)

        if 'pop_corn' in name:
            marker.type = Marker.SPHERE
            marker.scale.x = 0.040
            marker.scale.y = 0.040
            marker.scale.z = 0.040
            marker.color.a = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            ma.markers.append(marker)
            ma.markers.append(text_marker)
            pcl.points += gen_flatcircle(marker.scale.x / 2.0, marker.pose.position.x, marker.pose.position.y)

        if 'cup' in name:
            marker.type = Marker.CYLINDER
            marker.pose.position.z += 0.075
            marker.scale.x = 0.095
            marker.scale.y = 0.095
            marker.scale.z = 0.150
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            ma.markers.append(marker)
            ma.markers.append(text_marker)
            pcl.points += gen_flatcircle(marker.scale.x / 2.0, marker.pose.position.x, marker.pose.position.y)
        
        # if 'nucobot' in name:
        #     br.sendTransform((data.pose[i].position.x, data.pose[i].position.y, data.pose[i].position.z),
        #              (data.pose[i].orientation.x, data.pose[i].orientation.y, data.pose[i].orientation.z, data.pose[i].orientation.w),
        #              rospy.Time.now(),
        #              "base_footprint",
        #              "world")




    pub_pcl.publish(pcl)
    pub.publish(ma)






if __name__ == '__main__':
    rospy.init_node('fake_towermap', anonymous=False)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    rospy.spin()





