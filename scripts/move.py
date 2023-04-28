#!/usr/bin/env python

import rospy
import numpy as np
from states_new import IcuasStatesNew
from ball_release import ReleasePosition, MoveToTarget, ReleaseCallback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def main():
    intersection = None
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=0.1)
    pub_topp = rospy.Publisher('red/tracker/input_pose', PoseStamped, queue_size=0.1)
    pub_kill = rospy.Publisher("/red/kill_traj_planner", Bool, queue_size=0.1, latch=True)
    pub_release = rospy.Publisher('red/uav_magnet/gain', Float32, queue_size=1)
    pub_nn = rospy.Publisher("", Bool, queue_size=10)
    loop = 0
    pose = PoseStamped()

    while loop == 0: # First loop
        if cbr_states.return_objects('is_running') == True:
            pub_pose_stamped(7,0,3, 0,0,0,1, 'move_base_simple/goal')

            print("Loop = 1")

            loop = 1

    while loop == 1: # Stop ego-planner 
        if icuas_states.return_objects('position')[0] > 6.9:
            print("Loop = 2")
            pub_kill.publish(True) 
            loop = 2 


def pub_pose_stamped(x,y,z, q_x,q_y,q_z,q_w, topic):

    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=0.1)
    pub_topp = rospy.Publisher('red/tracker/input_pose', PoseStamped, queue_size=0.1)

    pose = PoseStamped()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w

    if topic == 'move_base_simple/goal':
        pub.publish(pose)
    elif topic == 'red/tracker/input_pose':
        pub_topp.publish(pose)
    else:
        pub.publish(pose)
        pub_topp.publish(pose)

def publish_nn(value):
    pub_nn = rospy.Publisher("", Bool, queue_size=10)

    pub_nn.publish(value)

if __name__ == '__main__':
    try:
        cbr_states = CBRStatesNew()
        main()
    except rospy.ROSInterruptException:
        pass
