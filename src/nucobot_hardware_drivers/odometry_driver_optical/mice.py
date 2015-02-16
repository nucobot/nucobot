#!/usr/bin/python
import rospy
from std_msgs.msg import Int32MultiArray
from evdev import InputDevice, list_devices, ecodes
from select import select
import sys, os

left_phys = "usb-0000:00:1a.0-1.2/input0"
rhgt_phys = "usb-0000:00:14.0-2/input0"
devices = [InputDevice(fn) for fn in list_devices()]

def print_all():
    for dev in devices:
        rospy.loginfo(dev.name+"\tpyhs = "+str(dev.phys))

def find_phys(ph):
    success = False
    for dev in devices:
        if ph == dev.phys:
            success = True
            return dev
    if (not success):
        rospy.logerr(str(ph) + " phys was not found!")
        exit(1)

def main():
    rospy.init_node('nucobot_mice_driver')
    print_all()

    left_mouse = find_phys(left_phys)
    rght_mouse = find_phys(rhgt_phys)

    left_fd = left_mouse.fd
    rght_fd = rght_mouse.fd

    valid_devices = [left_mouse, rght_mouse]
    valid_devices = {dev.fd : dev for dev in valid_devices}

    rospy.loginfo("\nUsing:")
    for dev in valid_devices.values(): rospy.loginfo(str(dev))

    for dev in valid_devices.values():
        dev.grab()

    pub = rospy.Publisher('nucobot_mice_driver/xy_shift', Int32MultiArray, queue_size=10)

    message = Int32MultiArray()
    for i in xrange(4): message.data.append(0)
    while not rospy.is_shutdown():
        for i in xrange(4): message.data[i] = 0
        r,w,x = select(valid_devices, [], [])
        for fd in r:
            for event in valid_devices[fd].read():
                if event.type == ecodes.EV_REL:
                    if (fd == left_fd):
                        if (event.code == ecodes.REL_X):
                            message.data[0] = event.value
                            #rospy.loginfo("xleft: --> " + str(event.value))
                        if (event.code == ecodes.REL_Y):
                            message.data[1] = event.value
                            #rospy.loginfo("yleft: --> " + str(event.value))
                    if (fd == rght_fd):
                        if (event.code == ecodes.REL_X):
                            message.data[2] = event.value
                            #rospy.loginfo("xrght: --> " + str(event.value))
                        if (event.code == ecodes.REL_Y):
                            message.data[3] = event.value
                            #rospy.loginfo("yrght: --> " + str(event.value))
        pub.publish(message)
    for dev in valid_devices.values():
        dev.ungrab()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("Shutting down odometry hardware driver(.py)")



