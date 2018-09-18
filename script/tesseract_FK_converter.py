#!/usr/bin/env python


import rospy, numpy


from tesseract_msgs.msg import Trajectory

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

joint_traj=None

def fnc_callback(msg):
    global joint_traj
    print('Got Tesseract Trajectory')
    joint_traj = msg.joint_trajectory   #This is type trajectory_msgs/JointTrajectory


if __name__=='__main__':


    rospy.init_node('FKConverterNode')

    sub=rospy.Subscriber('/trajopt/display_tesseract_trajectory', Trajectory, fnc_callback)
    pub_traj=rospy.Publisher('/converted_tesseract_trajectory', JointTrajectory, queue_size=1)
    pub_state=rospy.Publisher('/joint_states', JointState, queue_size=1)
    joint_state = JointState()



    while not rospy.is_shutdown():
        if joint_traj != None:
            pub_traj.publish(joint_traj)


            # Get joint names from trajectory
            joint_state.name = joint_traj.joint_names

            tmr = rospy.Duration()
            for ind in range(0,len(joint_traj.points)):
                # Fill out the header info
                joint_state.header.stamp = rospy.Time.now()

                # pull trajectory point info into joint state message
                joint_state.position = joint_traj.points[ind].positions
                joint_state.velocity = joint_traj.points[ind].velocities
                joint_state.effort = joint_traj.points[ind].effort

                pub_state.publish(joint_state)
                print('Publish /joint_states')

                # sleep for the neccessary time specified in the trajectory (Possible bug here)
                rospy.sleep(joint_traj.points[ind].time_from_start - tmr)
                tmr = joint_traj.points[ind].time_from_start

            rospy.sleep(1)




