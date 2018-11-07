# Robot Kinematics

Introduction 
---

classical mechanics of how things moves without regards to the forces that cause 

Forward kinematics the joint variables are known, calculate position and orientation of EE in cartesian cordinates

Inverse kinematics the pose is known, calculate joint variables need to be calculated

Degrees of freedom (DOF)

The minimum number of variables that are required to define the position or configuration of a mechanism in space. 

A single point that lies on a line needs only one coordinate to describe its position. 

That same point on a plane needs two coordniates to describe its motion (x,y)

Two points connect by a stiff rod needs 3 variables to describe it's position in space. (x, y, angle). 

## Two DoF Arm 

A system of two points can be constrained down to just one degree of freedom by connecting the points with a rod and anchoring the system at one end. 

If we add another (3rd point), we introduce a 2nd DoF, effectively creating a system of points that ressembles a robotic arm. 
Fixing p0 to the origin of the coordinate frame and specifying the length of each link introduced constraints to the robot.

Two additional parameters are need to completely describe the configuration of the system

0. Joint angle 1
1. Joint angle 2


# Excersie 

I am going to write a function in python that takes in the length of 2 linkks, the joint angles and outputs the (x,y) position of the joint at p1 and the end effector at p2. 

This function will be later implemented in a jupyter notebook for easier portability. For now let's explain some math that drives the code. 

![alt text][image] 

At the local frame 1 solving for the position of P1 with respect to the global frame at P0

x1 = l1 * cos(theta1)

y1 = l1 * sin(theta1)

At the local frame 2, solving for the position of P2 with respect to the global frame at P1

x2 = l2 * cos(theta1 + theta2)
y2 = l2 * sin(theta1 + theta2)

Combining the eaquations and solving for the position of the end effect P2 witht respect to the global frame at P0: 

x2 = l1 * cos(theta1) + l2 * cos(theta1 + theta2)

y2 = l1 * sin(theta1) + l2 * sin(theta1 + theta2) 


# Generalized Coordinates

The coordinates used to decribe the instantaneous configuration of a system.  


Previously we calculated the configuration of a 2-DoF manipulator. Now we will expand that generalize that concept to n-DoF systems of joints and links constrained to a plane. 

NOTE: With each new joint/link, there is a 1 DoF and 1 additional coordniate that is needed to fully describe the configuration of the system. 

DoF = # number of independent generalized coordinates

Configuration space or "Joint space" - is the set of all possible configurations a manipulator may have. This concept is extremely important for path planning and obstacle avoidance. 

## Rigid Bodies in Free Space 

For ANY rigid body moving on a plane:

If you locate one point on the body and define its orientation with respect to some fixed axis
then its configuration has been completely specified.

Moreover, the location of any other point on the rigid body can be located with knowledge of those three variables. 

6 coordinates are needed to fully describe the configuration of a rigid body in 3D free space. Thus it has 6 DoF.

Serial manipulator problems can be broken down in terms of rigid bodies connected by joints. 

NOTE: Constraining points with links reduces the degrees of freedom of a system.
NOTE: Adding joints to a system of otherwise disconnected bodies, reduces the degrees of freddom. 

Keep in mind
---
Rigid bodies have 6 degrees of freedom in space:

0. Position (x, y, z) 
1. Orientation (alpha, beta, gama) 

A point has 3 degrees of freedom in space

0. Position (x, y, z) 

# Joint types

![alt text][JointTypes] 

There are two commonly used joint types when dealing with serial manipulators

0. Revolute 
1. Prismatic

With only prismatic and/or revolute joints:

DoF = # of joints.
NOTE: The exception to this rule is when both ends of the manipulator are fixed!

If a manipulator has more DoF than is required for it's given task the system is Kinematically redundant or overconstrained.

Kinematically redundant manipulators are  more dexterous which means their end effector can reach more points with an arbitrary orientation and is better at avoiding obstacles. 

Path planning flexibilty means that they can also be more energetically efficient.
NOTE: Redundancy is more difficult to control 


# Principal types of serial manipulators

Classification is based on the kinematic config of arm, aka the sequence of joint types for the first 3 moveable links

Commonly used in industry 
---
Cartesian (PPP)
Cylindrical (RPP)
Anthropomorphic (RRR)
SCARA (RRP)
Spherical (RRP) 


# Spherical wrist
Is composed of three revolute joints whose aces of rotation all intersect at a common point. 

# Serial manipulator applications 

The workspace
---
The workspace is the set of all points reachable by the end effector and is a primary design contraint when selecting a manipulator for a job. 

Two regions

0. Reachable 
1. Detrous - all the points reachable by the end effector with an arbitrary orientation. 
NOTE: is a subset of the reachable workspace

---

Choosing a manipulator 
--- 

# Cartesian 
![alt text][Cartesian]

The first three joints of a Cartesian manipulator are prismatic joints with mutually orthogonal axes of translation.

Pros

Can have very high positional accuracy

Large payloads (gantry)

Simplest control strategy since there are no rotational movements

Very stiff structure

Cons:

All the fixtures and associated equipment must lie within its workspace

Requires large operating volume

Typical Applications:

Palletizing

Heavy assembly operations (e.g., cars and airplane fuselage)

# Cylindrical 

As the name suggests, the joints of a cylindrical manipulator are the cylindrical coordinates of the wrist center relative to the base.

Pros:

Large, easy to visualize working envelope

Relatively inexpensive for their size and payload

Cons:

Low average speed

Less repeatable than SCARA

Typical Applications:

Depends on the size, small versions used for precision assembly, larger ones for material handling, machine loading/unloading

# Anthropomorphic manipulator 

Anthropomorphic (sometimes called articulated) manipulators provide a relatively large workspace with a compact design. The first revolute joint has a vertical axis of rotation and can be thought of as mimicking a human’s ability to rotate at the waist. The other two revolute joints have axes of rotation that are perpendicular to the "waist" and mimic a one DoF “shoulder” and a one DoF “elbow”.

Pros:

Large workspace

Compact design

Cons:

Positional accuracy and repeatability is not as good as some other designs
Typical Applications:

Welding, spray painting, deburring, material handling


# SCARA 

The SCARA, or Selectively Compliant Assembly Robot Arm, was invented by Professor Hiroshi Makino of Yamanashi University (Japan) in the early 1980s. SCARA robots typically employ a single revolute wrist with the axis of rotation parallel to the other two revolute joints. Since the base link typically houses the actuators for the first two joints, the actuators can be very large and the moving links relatively light. Thus, very high angular speeds are obtainable with this design. The arm is very stiff in the vertical (z-axis), but relatively compliant in the x-y plane, which makes it ideal for tasks such as inserting pegs or other fasteners into holes.

Pros:

Fast

Compact structure

Cons:

Operations requiring large vertical motions
Typical Applications:

Precision, high-speed, light assembly within a planar environment


# Spherical 

Like the cylindrical manipulator, the spherical manipulator’s wrist center can also be described as a well-known coordinate system. Probably the best known version of this kinematic type is Stanford’s Scheinman arm, invented by Victor Scheinman in 1969.

It was adapted by manufacturers to become the leading robot in assembling and spot-welding products, ranging from fuel pumps and windshield wipers for automobiles to inkjet cartridges for printers.

Pros:

Large working envelope
Cons:

Complex coordinates more difficult to visualize, control, and program

Low accuracy

Relatively slow

Typical Applications:

Material handling

Spot welding

NOTE: Look up parallel manipulators



# Setting up the problem

# Coordinate Frames and Vectors

# Rotation Matrices in 2D

# Rotation Matrices in 3D

# Rotations in Sympy

# Composition of Rotations

# Euler Angles from a Rotation Matrix

# Translations 

# Homogeneous Transformations

# Composition of Homogenous Transforms

# Denavit-Hartenberg Parameters

# DH Parameter Assignment Algorithm 

# The 8 DH Steps

# DH Parameter Table 

# Forward Kinematics

# Inverse Kinematics

# Example

