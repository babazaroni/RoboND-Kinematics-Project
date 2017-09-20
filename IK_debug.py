from sympy import *
from time import time
from mpmath import radians
import numpy as np
import tf

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1: [[[2.16135, -1.42635, 1.55109],
                   [0.708611, 0.186356, -0.157931, 0.661967]],
                  [1.89451, -1.44302, 1.69366],
                  [-0.65, 0.45, -0.36, 0.95, 0.79, 0.49]],
              2: [[[-0.56754, 0.93663, 3.0038],
                   [0.62073, 0.48318, 0.38759, 0.480629]],
                  [-0.638, 0.64198, 2.9988],
                  [-0.79, -0.11, -2.33, 1.94, 1.14, -3.68]],
              3: [[[-1.3863, 0.02074, 0.90986],
                   [0.01735, -0.2179, 0.9025, 0.371016]],
                  [-1.1669, -0.17989, 0.85137],
                  [-2.99, -0.12, 0.94, 4.06, 1.29, -4.12]],
              4: [],
              5: []}



def IK_exec(req):
    joint_trajectory_list = []
    return joint_trajectory_list

def error_check(WC,FK,positions,test_case,start_time):
    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0], WC[1], WC[2]]  # <--- Load your calculated WC values in this array
    your_ee = [FK[0,3], FK[1,3], FK[2,3]]  # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time() - start_time))

    # Find WC error
    if not (sum(your_wc) == 3):
        wc_x_e = abs(your_wc[0] - test_case[1][0])
        wc_y_e = abs(your_wc[1] - test_case[1][1])
        wc_z_e = abs(your_wc[2] - test_case[1][2])
        wc_offset = sqrt(wc_x_e ** 2 + wc_y_e ** 2 + wc_z_e ** 2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(positions[0] - test_case[2][0])
    t_2_e = abs(positions[1] - test_case[2][1])
    t_3_e = abs(positions[2] - test_case[2][2])
    t_4_e = abs(positions[3] - test_case[2][3])
    t_5_e = abs(positions[4] - test_case[2][4])
    t_6_e = abs(positions[5] - test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not (sum(your_ee) == 3):
        ee_x_e = abs(your_ee[0] - test_case[0][0][0])
        ee_y_e = abs(your_ee[1] - test_case[0][0][1])
        ee_z_e = abs(your_ee[2] - test_case[0][0][2])
        ee_offset = sqrt(ee_x_e ** 2 + ee_y_e ** 2 + ee_z_e ** 2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)

    return




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
#        wc_pos2 = ee_pos - s[d7] * rm_ee[:, 2] - Matrix([[s[a1]],[0],[s[d1]]])

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
            theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
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


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0

    class Position:
        def __init__(self, EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]

    class Orientation:
        def __init__(self, EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self, position, orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position, orientation)

    class Pose:
        def __init__(self, comb):
            self.poses = [comb]

    req = Pose(comb)

    IK_exec(req,test_case = test_case)





if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 2

    test_code(test_cases[test_case_number])
