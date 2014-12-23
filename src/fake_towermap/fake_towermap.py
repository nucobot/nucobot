#!/usr/bin/env python

import re
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from gazebo_msgs.msg import ModelStates


pub = rospy.Publisher('visualization/fake_towermap/objects', MarkerArray, queue_size=10)

       


def callback(data):
    ma = MarkerArray()
    


    for i, name in enumerate(data.name):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.ns = "toys"
        marker.header.stamp = rospy.Time()
        marker.action = Marker.ADD

        if 'cylinder' in name:


            
            marker.type = Marker.CYLINDER
            marker.pose = data.pose[i]
            marker.id = 10 * marker.type + int(re.split('_*', name)[-1])
 
            rospy.loginfo(rospy.get_caller_id() + " " +name+ " " + str(marker.id))
            rospy.loginfo(rospy.get_caller_id() + " " + str(marker.pose))
            
            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.07
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            ma.markers.append(marker)
            #rospy.loginfo(rospy.get_caller_id() + marker.ns)

    
    for m in ma.markers:
        rospy.loginfo(rospy.get_caller_id() + " " + str(len(ma.markers)))
        rospy.loginfo(rospy.get_caller_id() + " " + str(m.id))
        rospy.loginfo(rospy.get_caller_id() + " " + str(m.pose))

    pub.publish(ma)
    

if __name__ == '__main__':
    rospy.init_node('fake_towermap', anonymous=False)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    rospy.spin()
