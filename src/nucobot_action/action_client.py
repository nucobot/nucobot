#!/usr/bin/env python

import roslib; roslib.load_manifest('nucobot_action')
import rospy, smach, smach_ros


from nucobot_action.msg import *
from actionlib import *
from actionlib.msg import *


# TODO: replace with parameter server
movement_speed = 0.0


# Ask user whether we want to launch the robot or not
class PauseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue', 'abort'])

    def execute(self, userdata):
        rospy.loginfo("PauseState: press 'y' to continue or 'n' to abort...")

        char = raw_input()
        while char.lower() not in ("y", "n"):
            rospy.loginfo("Please, choose 'y' or 'n'")
            char = raw_input()
        
        if char == 'y':
            return 'continue'
        return 'abort'


def main():
    rospy.init_node('nucobot_action_client')


    sm0 = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with sm0:
        smach.StateMachine.add('Pause', PauseState (),
                               transitions={'continue':'Achieve target',
                                            'abort'   :'aborted'})

        smach.StateMachine.add('Achieve target',
                               smach_ros.SimpleActionState('AchieveTargetAS',
                                                           AchieveTargetAction,
                                                           goal =  AchieveTargetGoal(vel=movement_speed),
                                                           outcomes=['aborted', 'succeeded']),
                               transitions={'aborted'   :'aborted',
                                            'succeeded' :'Pause'} )


    # Create and start the introspection server
    # This is for debug purpose
    # sis = smach_ros.IntrospectionServer('introspection_server', sm0, '/STATE_MASHINE')
    # sis.start()


    # Execute SMACH plan
    outcome = sm0.execute()
    print("State mashine has finished with result ", outcome)

    rospy.spin()
    #sis.stop()

if __name__ == '__main__':
    main()