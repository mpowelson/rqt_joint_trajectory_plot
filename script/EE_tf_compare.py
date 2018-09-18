#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import json
import numpy as np

if __name__ == '__main__':
    rospy.init_node('EE_tf_compare')

    listener = tf.TransformListener()

#    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    # Import json file as a python dictionary
    f = open('/home/mpowelson/workspaces/trajopt/src/trajopt_ros/trajopt_examples/config/basic_cartesian_plan_confined_axis.json')
    dict = json.load(f)
    waypoints = len(dict.get('constraints'))

    rate = rospy.Rate(10.0)

    y_rot = np.zeros((waypoints))
    for ind in range(0,waypoints):
        xyz  = dict.get('constraints')[ind].get('params').get('xyz')    # There is probably shorthand for this.
        wxyz = dict.get('constraints')[ind].get('params').get('wxyz')
        confined_coef = dict.get('constraints')[ind].get('params').get('confined_coeff')
        tmp = tf.transformations.euler_from_quaternion(wxyz)[1]*180./3.14159
        y_rot[ind] = tmp

    print('Read Contraints: Targets')
    print(y_rot)


    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/link_7', rospy.Time(0))
            rot_euler = tf.transformations.euler_from_quaternion(rot)
#            print( (( y_rot - rot_euler[1])*180./3.14159)%360)
            print(rot_euler[1]*180./3.14159)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue




#        angular = 4 * math.atan2(trans[1], trans[0])
#        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
#        cmd = geometry_msgs.msg.Twist()
#        cmd.linear.x = linear
#        cmd.angular.z = angular
#        turtle_vel.publish(cmd)

        rate.sleep()
