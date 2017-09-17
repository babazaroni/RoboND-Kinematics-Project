## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/joint-diagram.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The joint diagram describing the KR210 manipulator is:

![alt text][image4]

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2	 | -0.054	 | 1.5 | q4
4->5 | pi/2	 | 0 | 0 | q5
5->6 | -pi/2	 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The individual transformation matrices about each joint can be constructed by first defining the a function that creates the matrix and substitues the values from the DH table defined above.  Then, each joint to joint table can be defined by calling this function with the corresponding symbols and supplying the DH table.

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
    T6_G=  make_T(alpha6,a6,d7,q7,DH_Table)
```

Here is the generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose. R is roll, P is pitch, Y is yaw.  px,py,pz is the gripper position.

```Matrix([
[cos(p)*cos(Y), sin(p)*sin(r)*cos(Y) - sin(y)*cos(R), sin(P)*cos(R)*cos(Y) + sin(R)*sin(Y), px],
[sin(y)*cos(P), sin(p)*sin(r)*sin(Y) + cos(r)*cos(Y), sin(P)*sin(Y)*cos(R) - sin(R)*cos(Y), py],
[      -sin(P),                        sin(r)*cos(P),                        cos(P)*cos(R), pz],
             0,                                    0,                                    0,  1]])
```




#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


