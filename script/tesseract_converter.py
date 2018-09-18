#!/usr/bin/env python


#Converts from tesseract format so we can use the rqt plugin to view trajectories

#remove or add the library/libraries for ROS
import rospy   #, time, math, cv2, sys


from tesseract_msgs.msg import Trajectory

from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
#from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

varS=None

def fnc_callback(msg):
    global varS
    print('Got Tesseract Trajectory')
    varS = msg.joint_trajectory   #This is type trajectory_msgs/JointTrajectory


if __name__=='__main__':


    rospy.init_node('ConverterNode')

    sub=rospy.Subscriber('/trajopt/display_tesseract_trajectory', Trajectory, fnc_callback)
    pub=rospy.Publisher('/converted_tesseract_trajectory', JointTrajectory, queue_size=1)
    rate=rospy.Rate(1)

    while not rospy.is_shutdown():
        if varS != None:
            pub.publish(varS)
            print('pub')

        rate.sleep()
