#!/usr/bin/env python

import roslib
import rospy
import actionlib


#move_base_msgs
from move_base_msgs.msg import *
from scitos_msgs.msg import *

def simple_move():
    print("Starting move test")
    rospy.init_node('move_node')
    sac = actionlib.SimpleActionClient('move_base_path', MoveBasePathAction )
    goal1 = MoveBasePathGoal()

    #set goal
    #goal1.target_poses.header.frame_id = 'base_link'
    goal1.target_poses.pose.position.x = 2.0
    goal1.target_poses.pose.position.y = 1.0
    goal1.target_poses.pose.orientation.w = 0.0
    #goal1.target_poses.header.stamp = rospy.Time.now()
    print("Waiting for server")
    sac.wait_for_server()
    print("Sending command")
    sac.send_goal(goal1)
    print("Waiting for result")
    sac.wait_for_result()
    print sac.get_result()


if __name__ == '__main__':
    try:
        simple_move()
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
