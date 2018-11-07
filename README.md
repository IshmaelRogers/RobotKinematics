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

I am going to write a function in pythin that takes in the length of 2 linkks, the joint angles and outputs the (x,y) position of the joint at p1 and the end effector at p2. 











Reference frames 

Generalized Coordinates

Joint types

Principal types of serial manipulators

Serial manipulator applications 

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

