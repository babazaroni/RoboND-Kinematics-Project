
[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/joint-diagram.png
[image5]: ./misc_images/SSS-diagram.png
[image6]: ./misc_images/joint-diagram-hand-drawn.jpg
[image7]: ./misc_images/j1234-hand-drawn.jpg

The joint diagram describing the KR210 manipulator and each joint's z and x axis:

![alt text][image4]

Below is a hand drawn diagram listing the locations of the `alpha`, `a` and `d` constants and the `q` variables.  The alphas represent the orientation of a joint's z axis from the previous joint's z axis.  The a's are the displacement of the joint's z axis from the previous joint's z axis along the x axis.  The `d` constant is the displacement of the joint's x axis from the prevous joint x axis along the z axis.  The theta's represent the joint's angle of movement.

![alt text][image6]

The constants and variables in table form:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2	 | -0.054	 | 1.5 | q4
4->5 | pi/2	 | 0 | 0 | q5
5->6 | -pi/2	 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

The constants in the table were derived from values in the the manufacturere supplied kr210.urdf.xacro file as follows:

(The values in the urdf file are relative to previous joint)

The alpha values describe the orientation of a joint's Z axis relative to the previous joint's Z axis and are either 0, pi/2, or -pi/2 radions.  (0,90,-90 degrees).  They are derived from the unit vectors described in the axis tag.

The axis choices are defined by the DH convention choices and don't always match the manufacturers choice.

The 'd' values are the displacement of the joints z axiz as measured along the x azis

```
a1:  .35  x value of Joint 2 position (line 330)
a2: 1.25  z value of Joint 3 position (line 337)
a3: -.54  z value of Joint 4 position (line 344)
```

The 'd' values are the displacement of the joints x axiz as measured along the z azis

```
d1:  .75  z value of Joint 1 position (line 323) + z value of Joint 2 position (line 330)
d4: 1.50  x value of Joint 4 position (line 344) + x value of Joint 5 position (line 351)
d7: .303  x value of Gripper Joint position (line 288) + x value of Joint 6 position (line 358)
```

A dictionary of these constants and variables can be constructed as follows:

```
DH_Table = {
        alpha0:       0, a0:      0, d1: 0.75, q1:          q1,
        alpha1: -pi / 2, a1:   0.35, d2:    0, q2: q2 - pi / 2,
        alpha2:       0, a2:   1.25, d3:    0, q3:          q3,
        alpha3: -pi / 2, a3: -0.054, d4: 1.50, q4:          q4,
        alpha4:  pi / 2, a4:      0, d5:    0, q5:          q5,
        alpha5: -pi / 2, a5:      0, d6:    0, q6:          q6,
        alpha6:       0, a6:      0, d7:0.303, q7:           0 }
```



The individual transformation matrices about each joint can be constructed by first defining a function that creates the matrix and substitues the values from the DH table defined above.  Then, each joint to joint table can be defined by calling this function with the corresponding symbols and supplying the DH table.

```
    def make_T(twist_angle, link_length, link_offset, joint_angle, subvals):
        m = Matrix([[cos(joint_angle), -sin(joint_angle), 0, link_length],
                    [sin(joint_angle) * cos(twist_angle), cos(joint_angle) * cos(twist_angle), -sin(twist_angle),
                     -sin(twist_angle) * link_offset],
                    [sin(joint_angle) * sin(twist_angle), cos(joint_angle) * sin(twist_angle), cos(twist_angle),
                     cos(twist_angle) * link_offset],
                    [0, 0, 0, 1]])
        return m.subs(subvals)

    # Create individual transformation matrices

    T0_1 = make_T(alpha0,a0,d1,q1,DH_Table)
    T1_2 = make_T(alpha1,a1,d2,q2,DH_Table)
    T2_3 = make_T(alpha2,a2,d3,q3,DH_Table)
    T3_4 = make_T(alpha3,a3,d4,q4,DH_Table)
    T4_5 = make_T(alpha4,a4,d5,q5,DH_Table)
    T5_6 = make_T(alpha5,a5,d6,q6,DH_Table)
    T6_G = make_T(alpha6,a6,d7,q7,DH_Table)
```

Here is the generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose. `R` is roll, `P` is pitch, `Y` is yaw.  `px,py,pz` is the gripper position.

```

[cos(p)*cos(Y), sin(p)*sin(r)*cos(Y) - sin(y)*cos(R), sin(P)*cos(R)*cos(Y) + sin(R)*sin(Y), px]
[sin(y)*cos(P), sin(p)*sin(r)*sin(Y) + cos(r)*cos(Y), sin(P)*sin(Y)*cos(R) - sin(R)*cos(Y), py]
[      -sin(P),                        sin(r)*cos(P),                        cos(P)*cos(R), pz]
             0,                                    0,                                    0,  1]

```


### Inverse Kinematics

The last three joints (4,5,6) form a wrist such that the center of joint 5 is the wrist center.  No matter what adjustments you make to joint 4,5, or 6 the position in world coordinates of the wrist center will not change.  The wrist center position is dependant only on joints 1,2,3.  So we can seperate the inverse kinematics problem, one to find the joint angles 1,2,3 from the wrist position, and the second to find the joint angles 4,5,6 from the orientation of the gripper.  The orientation is it's roll, pitch and yaw.

#### Inverse Position Kinematics


![alt text][image7]

The diagram above is used to calculate the joint positions (thetas) of the first three joints.

For theta1, we use the left diagram which is a view down on the arm as it is seen on the xy plane.  The calculation is thus:

```
        theta1 = atan2(wcy,wcx)
```


The diagram on the right above is a view of the plane formed by the joints.  To calculate the joint angles, we will need to find the angles `a,b`.  This requires us to obtain the lengths `A,B,C`.  These sides form an SSS triangle and we can use the law of cosines to find the angles

`A` is a the constant distance between joint 3 and the wrist center.  It is is measured with RViz and set accordingly.
`C` is the constant distance between joint 2 and 3.

To find `B` we need to calculate `D` and `E`.  Here is the code

```
        D = sqrt(wcx * wcx + wcy * wcy) - s[a1]
        E = wcz - s[d1]

        A = 1.501  # use RVIZ to measure the distance between joint 3 and wrist center
        B = sqrt(D * D + E * E)
        C = s[a2]  # x distance between joint 1 and 2
        

        a = acos((B * B + C * C - A * A) / (2 * B * C))
        b = acos((A * A + C * C - B * B) / (2 * A * C))
```

`Theta2` is the remaining angle after you subtract angle `a` plus the angle formed by side `B` and the x axis from 90 degrees.
Link 4 sags by a small fixed amount (.036 rads) so `theta3` is the remainder of angle `b` and the `sag angle` subtraced from 90 degrees.

```

        theta2 = pi / 2 - a - atan2(E, D)
        theta3 = pi / 2 - angle_b - .036  # .036 accounts for sag in link4 of -.054m
        
```

#### Inverse Orientation Kinematics

Since we have uncoupled the inverse orientation kinemtics we can focus on the last three joints. The key to finding theta 4,5,6 is to obtain the rotation matrix for joints 4,5,6.  Since we are given the pose of the end effector, we know the rotation matrix for the base to end effector (global rotation matrix).  The rotation matrix for the last three joints is then extracted from the global rotation matrix by inverting the rotation matrix of the first three joints and multiplying by the global rotation matrix.


        

        R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

        # The rotation matrix from 3 to 6 is total rotation to the end effector with the 0-3 rotation extracted by
        # multiply with the 0-3 inverse.  Transpose can give inverse for symetric matrics

        R3_6 = R0_3.transpose() * rm_ee  # transpose gives better results than inv("LU")
        
        theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
        theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
        theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])


### Project Summary

The task was to code handle_calculate_IK routine.  Gazebo first plans a route the robot arm should take to get the end effector to a position in front of the target and a second time, move to the bin.  The plan is a list of positions and orientations of the end effector.  This list is passed to handle_calculate_IK and it returns a list of joint positions for each of the points and orientations in the list.

#### Tasks that are done once for each call handle_calculate_IK

1. Create the symbols to represent the `alpha`, `a`, `d`, and `q` values.
2. Create the dh table dictionary `s` to hold the values
3. Define function `make_T' to create a transformation table and substitute in values from the 's' dictionary.
4. Call the function 'make_T' to create the transformation tables

#### Tasks that are done for each end effector pose

1.   Extract the end effector position from the pose into `ee_pos`.
2.   Extract the `roll`, `pitch` and `yaw` from the pose orientation using the `tf.transformations.euler` method
3.   Create the `rm_roll`, `rm-pitch` and `rm_yaw` roll matrices.
4.   Calculate the global rotation matrix `rm_ee` from the roll, pitch and yaw matrices.
5.   Adjust `rm_ee` to compensate for the difference in orientation defined in urdf file and our dh convention
6.   Substitue into `rm_ee` the values of roll, pitch, and yaw from the current pose.
7.   Calculate the wrist position.
8.   Calculate the theta's for the first three joints.  Described in detail above.
9.   Calculate the theta's for the last three joints.  Described in detail above.
10.  Create a `JointTrajectoryPoint` from the theta's and place in the `joint_trajectory_list`.
11.  Return the `joint_trajectory_list`.


#### Results

The system takes about six seconds to return the joint list on a 2011 mac pro.  This could be improved by moving more calculations out of the inner loop.  But, it takes thirty seconds for the arm to move to the bin which is beyond the control of IK_server.py.  I noticed that there were a lot of unnecessary wrist movements during the movement to the bin so I eliminated them by only using the last wrist positions on the last pose request.  Some times, the motion planner would request a wild loop to the target or bin, which also is beyond the control of IK_server.py.  It would be interesting to see the planner code and to correct this.



