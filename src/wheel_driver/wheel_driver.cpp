#include <ros/ros.h>
#include <std_msgs/String.h>
#include <wheel_driver/WheelState.h>


void chatterCallback(const wheel_driver::WheelState& msg)
{
  ROS_INFO("I heard: [%g]", msg.wheel_0.x);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "wheel_driver");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<wheel_driver::WheelState>("wheel_state", 1000);
  ros::Subscriber sub = nh.subscribe("wheel_state", 1000, chatterCallback);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int i = 0;
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    wheel_driver::WheelState msg;

    msg.wheel_0.x = i++;

    ROS_INFO("%g", msg.wheel_0.x);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
