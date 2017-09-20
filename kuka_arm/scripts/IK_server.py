#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya and Chuck Carlson

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from time import time


#define an error and exec routine to make the copy/paste from ID_debug easier

def error_check(WC,FK,positions,test_case,start_time):
    return

# define IK_exec so that it is easy to copy/paste code back and forth between IK_debug.py and IK_server.py

def IK_exec(req,test_case = None):

    start_time = time()

    # Define DH param symbols
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  # link offset
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  # link length
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  # twist angle

    # Joing angle symbols

    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

    # the DH parameters


    s = {
        alpha0: 0, a0: 0, d1: 0.75, q1: q1,
        alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
        alpha2: 0, a2: 1.25, d3: 0, q3: q3,
        alpha3: -pi / 2, a3: -0.054, d4: 1.50, q4: q4,
        alpha4: pi / 2, a4: 0, d5: 0, q5: q5,
        alpha5: -pi / 2, a5: 0, d6: 0, q6: q6,
        alpha6: 0, a6: 0, d7: 0.303, q7: 0}



    # define routine with transformation template

    def make_T(twist_angle, link_length, link_offset, joint_angle, subvals):
        m = Matrix([[cos(joint_angle), -sin(joint_angle), 0, link_length],
                    [sin(joint_angle) * cos(twist_angle), cos(joint_angle) * cos(twist_angle), -sin(twist_angle),
                     -sin(twist_angle) * link_offset],
                    [sin(joint_angle) * sin(twist_angle), cos(joint_angle) * sin(twist_angle), cos(twist_angle),
                     cos(twist_angle) * link_offset],
                    [0, 0, 0, 1]])
        return m.subs(subvals)

    # Create individual transformation matrices

    T0_1 = make_T(alpha0,a0,d1,q1,s)
    T1_2 = make_T(alpha1,a1,d2,q2,s)
    T2_3 = make_T(alpha2,a2,d3,q3,s)
    T3_4 = make_T(alpha3,a3,d4,q4,s)
    T4_5 = make_T(alpha4,a4,d5,q5,s)
    T5_6 = make_T(alpha5,a5,d6,q6,s)
    T6_G=  make_T(alpha6,a6,d7,q7,s)

    # simplifying takes minutes so don't do
    #    T0_2 = simplify(T0_1 * T1_2) # base_link to link_2
    #    T0_3 = simplify(T0_2 * T2_3)  # base_link to link_2
    #    T0_4 = simplify(T0_3 * T3_4)  # base_link to link_2
    #    T0_5 = simplify(T0_4 * T4_5)  # base_link to link_2
    #    T0_6 = simplify(T0_5 * T5_6)  # base_link to link_2
    #    T0_G = simplify(T0_6 * T6_G)  # base_link to link_2




    # Initialize service response
    joint_trajectory_list = []
    for x in xrange(0, len(req.poses)):

        # extract end effector position from request

        ee_pos = Matrix([[req.poses[x].position.x],[req.poses[x].position.y],[req.poses[x].position.z]])


        # calculate the roll, pitch, yaw from the orientation in the request

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w]
        )

        # Calculate End Effector Rotation Matrices

        r, p, y = symbols('r p y')

        rm_roll = Matrix([[1, 0, 0],
                           [0, cos(r), -sin(r)],
                           [0, sin(r), cos(r)]])

        rm_pitch = Matrix([[cos(p), 0, sin(p)],
                            [0, 1, 0],
                            [-sin(p), 0, cos(p)]])

        rm_yaw = Matrix([[cos(y), -sin(y), 0],
                          [sin(y), cos(y), 0],
                          [0, 0, 1]])

        rm_ee = rm_yaw * rm_pitch * rm_roll

        #rm_error accounts for the difference in orientation in the urdf file and the our DH conventions.
        # need to rotate around x axis by -90 degrees and around z axis by 180 deg

        rm_error = rm_yaw.subs(y, radians(180)) * rm_pitch.subs(p, radians(-90))

        rm_ee = rm_ee * rm_error
        rm_ee = rm_ee.subs({'r': roll, 'p': pitch, 'y': yaw})


        # The wrist center position is the end effector position minus the offset to the wrist center

        wc_pos  = ee_pos - s[d7] * rm_ee[:, 2]
        wcx,wcy,wcz = wc_pos[0],wc_pos[1],wc_pos[2]

        # Calculate the joint angles
        # use geometric method

        theta1 = atan2(wcy,wcx)

        # We now have an SSS triangle and can use law of cosines to calculate the angles

        # See diagram in md file which shows the following segments

        D = sqrt(wcx * wcx + wcy * wcy) - s[a1]
        E = wcz - s[d1]

        A = 1.501  # use RVIZ to measure the distance between joint 3 and wrist center
        B = sqrt(D * D + E * E)
        C = s[a2]  # x distance between joint 1 and 2


        a = acos((B * B + C * C - A * A) / (2 * B * C))
        b = acos((A * A + C * C - B * B) / (2 * A * C))

        theta2 = pi / 2 - a - atan2(E, D)
        #link4 sags.  Calculate with atan2(0.054,sqrt(1.501**2 - 0.054**2))
        sag_angle = .036
        theta3 = pi / 2 - b - sag_angle

        # to calculate the last three theta angles, we need the rotation matrix from 3 to 6
        # extract and multiply the rotation matrices from first 3 transformation matrices
        # then substitue in the first three joint angles to get the rotation matrix from 0 to 3

        R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

        # The rotation matrix from 3 to 6 is total rotation to the end effector with the 0-3 rotation extracted by
        # multiply with the 0-3 inverse.  Transpose can give inverse for symetric matrics

        R3_6 = R0_3.transpose() * rm_ee  # transpose gives better results than inv("LU")

        if x == len(req.poses) - 1:  # use only the last wrist angles
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt( pow(R3_6[0, 2],2) + pow(R3_6[2, 2],2) ), R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
        else:
            theta4 = 0  # this eliminates the unnecessary wrist rotations in the beginning
            theta5 = 0
            theta6 = 0

        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

        if test_case is not None:

            T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

            FK = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

            error_check(wc_pos,FK,joint_trajectory_point.positions,test_case,start_time)

    return joint_trajectory_list



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        joint_trajectory_list = IK_exec(req)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
