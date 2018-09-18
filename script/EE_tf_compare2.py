#!/usr/bin/env python

# Publishes the goal poses in the json as a pose array. Then yse the joint state publisher to get the TF, so you can see the TF
# move on top of the goals to compare

import rospy, tf
import json
import numpy as np
from tesseract_msgs.msg import Trajectory

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray

joint_traj=None

def fnc_callback(msg):
    global joint_traj
    print('Got Tesseract Trajectory')
    joint_traj = msg.joint_trajectory   #This is type trajectory_msgs/JointTrajectory


if __name__=='__main__':
    # ROS initializationtesseract_FK_converter
    rospy.init_node('EE_tf_compare')

    listener = tf.TransformListener()
    sub=rospy.Subscriber('/trajopt/display_tesseract_trajectory', Trajectory, fnc_callback)
    pub_traj=rospy.Publisher('/converted_tesseract_trajectory', JointTrajectory, queue_size=1)
    pub_state=rospy.Publisher('/joint_states', JointState, queue_size=1)
    pub_goal_pose=rospy.Publisher('/goal_pose', PoseArray, queue_size=10)
    joint_state = JointState()
    goal_pose_array = PoseArray()


    # Import json file as a python dictionary
    f = open('/home/mpowelson/workspaces/trajopt/src/trajopt_ros/trajopt_examples/config/basic_cartesian_plan.json')
    dict = json.load(f)
    num_pts = len(dict.get('constraints'))

    y_rot = np.zeros((num_pts))
    waypoints = np.zeros((num_pts,7))

    for ind in range(0,num_pts):
        xyz  = dict.get('constraints')[ind].get('params').get('xyz')    # There is probably shorthand for this.
        wxyz = dict.get('constraints')[ind].get('params').get('wxyz')
        confined_coef = dict.get('constraints')[ind].get('params').get('confined_coeff')
        tmp = tf.transformations.euler_from_quaternion(wxyz)[1]*180./3.14159
        y_rot[ind] = tmp
        waypoints[ind,0:3] = xyz
        waypoints[ind,3:] = wxyz
        goal_pose = Pose()   #workaround since python passes the pose by reference. Find correct way to do this rather than deletng
        goal_pose.position.x = waypoints[ind,0]
        goal_pose.position.y = waypoints[ind,1]
        goal_pose.position.z = waypoints[ind,2]
        goal_pose.orientation.w = waypoints[ind,3]
        goal_pose.orientation.x = waypoints[ind,4]
        goal_pose.orientation.y = waypoints[ind,5]
        goal_pose.orientation.z = waypoints[ind,6]
#        print(goal_pose)
        goal_pose_array.poses.append(goal_pose)
        del goal_pose
#        print(ind)
#        print(goal_pose_array)
        goal_pose_array.header.stamp = rospy.Time.now()
        goal_pose_array.header.frame_id = 'world'
    print(waypoints)


    flag = 0
    while not rospy.is_shutdown():
        if joint_traj != None:
            pub_traj.publish(joint_traj)
            print('Pub Traj')

            # Publish Goal Pose for visualization
            pub_goal_pose.publish(goal_pose_array)


            # Get joint names from trajectory
            joint_state.name = joint_traj.joint_names

            tmr = rospy.Duration()
            for ind in range(0,num_pts):   #Add check if traj is same length as constraints
#                print(ind)
                # Fill out the header info
                joint_state.header.stamp = rospy.Time.now()

                # pull trajectory point info into joint state message
                joint_state.position = joint_traj.points[ind].positions
                joint_state.velocity = joint_traj.points[ind].velocities
                joint_state.effort = joint_traj.points[ind].effort

                # Publish Joint State
                pub_state.publish(joint_state)
#                print('Publish /joint_states')
                if ind==0:
                    rospy.sleep(1)





                # sleep for the neccessary time specified in the trajectory (Possible bug here)
                rospy.sleep(joint_traj.points[ind].time_from_start - tmr)
                tmr = joint_traj.points[ind].time_from_start

                # Get transform (robot_state_publisher should have published by now). This setup is sloppy
                try:
                    (trans,rot) = listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
                    rot_euler = tf.transformations.euler_from_quaternion(rot, axes='sxyx')
                    delta = ((rot_euler[1] - waypoints[ind,1] )*180/np.pi)%360
                    if delta > 180:
                        delta -=360
                    print(delta, (rot_euler[1]*180/np.pi)%360, (waypoints[ind,1]*180/np.pi)%360)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            rospy.sleep(1)




