[image1]: ./Images/2DofRobot.PNG
[image2]: ./Images/Arm_config_diagram.PNG
[image3]: ./Images/2DofRobot2.PNG
[image4]: ./Images/JointTypes.PNG
[image5]: ./Images/cartesian_ws.PNG
[image6]: ./Images/cylindrical_ws.PNG
[image7]: ./Images/Anthoromorphic.PNG
[image8]: ./Images/SCARA.PNG
[image9]: ./Images/Spherical.PNG
[image10]: ./Images/DotProduct.PNG
[image11]: ./Images/reference_frame_B.PNG
[image12]: ./Images/VwrtAandB.PNG
[image13]: ./Images/2Drotation1.PNG
[image14]: ./Images/Basisvector.PNG
[image15]: ./Images/projection.PNG
[image151]: ./Images/3Drotation.PNG
[image152]: ./Images/Rotxyz.PNG
[image153]: ./Images/Extrinsic.PNG
[image154]: ./Images/intrinsic.PNG
[image155]: ./Images/Example1.PNG
[image156]: ./Images/solve4eulerangles.PNG
[image157]: ./Images/Translation.PNG
[image16]: ./Images/RotandTrans.png
[image17]: ./Images/Homogenous0.png
[image18]: ./Images/Homogenous1.png
[image19]: ./Images/example.png
[image20]: ./Images/inverseTransform.png
[image21]: ./Images/Homogenous_Comp.png
[image22]: ./Images/DHPARAMETERS.PNG
[image23]: FKandIK 
[image24]: totaltransform 
      

Robot Kinematics
---
# Introduction 


Kinematic belong to classical mechanics and is the of how things moves without regards to the forces that cause it to move. 

For serial manipulators there are two types of Kinematics that we focus on:

0. Forward
1. Inverse 

0. In Forward Kinematics the joint variables are known, and the taske is to calculate the position and orientation of the end effector in cartesian cordinates. 

1. Inverse Kinematics involves previous knowledge about the pose and the task is to calculate joint variables need to be to acheive that pose. 

# Degrees of freedom (DOF)

The minimum number of variables that are required to define the position or configuration of a mechanism in space. 

A single point that lies on a line needs only one coordinate to describe its position. 

That same point on a plane needs two coordniates to describe its motion (x,y)

Two points connected by a stiff rod needs 3 variables to describe it's position in space. 

(x, y, angle)

## Two DoF Arm 

![alt text][image1]

Consider a system of two points, this system can be constrained down to just one degree of freedom by connecting the points with a rod and anchoring the system at one end. 

Adding a 3rd point, would introduce a 2nd DoF system; 

![alt text][image3]

Creating a system of points that ressembles a robotic arm. 

Fixing p0 to the origin of the coordinate frame and specifying the length of each link introduced constraints to the robot.

From here only 2 additional parameters are need to completely describe the configuration of the system

0. Joint angle 1
1. Joint angle 2

# Excersie 

I am going to write a function in python that takes in the length of 2 linkks, the joint angles and outputs the (x,y) position of the joint at p1 and the end effector at p2. 

This function will be later implemented in a jupyter notebook for easier portability. For now let's explain some math that drives the code. 

![alt text][image2] 

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

![alt text][image4] 

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
![alt text][image5]

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
![alt text][image6]

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
![alt text][image7]

Anthropomorphic (sometimes called articulated) manipulators provide a relatively large workspace with a compact design. The first revolute joint has a vertical axis of rotation and can be thought of as mimicking a human’s ability to rotate at the waist. The other two revolute joints have axes of rotation that are perpendicular to the "waist" and mimic a one DoF “shoulder” and a one DoF “elbow”.

Pros:

Large workspace

Compact design

Cons:

Positional accuracy and repeatability is not as good as some other designs
Typical Applications:

Welding, spray painting, deburring, material handling


# SCARA 

![alt text][image8]

The SCARA, or Selectively Compliant Assembly Robot Arm, was invented by Professor Hiroshi Makino of Yamanashi University (Japan) in the early 1980s. SCARA robots typically employ a single revolute wrist with the axis of rotation parallel to the other two revolute joints. Since the base link typically houses the actuators for the first two joints, the actuators can be very large and the moving links relatively light. Thus, very high angular speeds are obtainable with this design. The arm is very stiff in the vertical (z-axis), but relatively compliant in the x-y plane, which makes it ideal for tasks such as inserting pegs or other fasteners into holes.

Pros:

Fast

Compact structure

Cons:

Operations requiring large vertical motions
Typical Applications:

Precision, high-speed, light assembly within a planar environment


# Spherical 
![alt text][image9]

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

A vector is a mathematical quantity that has both magnitude and direction.

Velocity, force position 

A coordnate frame aka the reference frame is where the vecotr can be graphically represented. 


In this example the vector v represents the position of point P on the A reference frame. 

v can be represented as:

0. Basis vector
1. An equation 

where v_x and v_y are called measure numbers because they measure how much of v is point in the direction defined by the unit vector ax_hat and ay_hat.

v_x and v_y are the magnitudes of v acting in the a_x and a_y directions. 

Q: How to determine numerical values for v_x and v_y? 
A: We project v onto ax_hat and ay_hat using the dot product operator. 

[dot product image]

NOTE: Since cosine is adjacent/hypotenuse, vx = v*cos(theta)

It is possible to have multiple reference frames. We will draw a new reference frame, B and describe vector v ( and ultimately the position of p) relative to it. 

NOTE: vector v has the exact same magnitude in reference frame B that it had in reference frame A.  

![alt text][image10]

In general, all we are doing is projecting vectors expressed in one ferame onto some other frame with rotation matrices. 

# Rotation Matrices in 2D

There are to conceptual interpertations of rotation matrices:

0. The means for expressing a vector in one coordinate frame in terms of some other coordinate frame. 
NOTE: This is known as mapping between frames
1. Can be viewed as an operator that moves a vector within a single coordinate frame. 
NOTE: Although the decription of these two interpertations are different, the mathematics works out to be the exact same. 

![alt text][image11]

How to express v with respect to reference frames A and B

![alt text][image12]


simplifing equation 5 will provide us with the following:

![alt text][image13]

The first term on the right is the rotation matrix. 

The columns of the rotation matrix are the basis vectors of B expressed in terms of A

![alt text][image14] 

The rows of the rotation matrix are the projection of the A frame onto the B frame 

![alt text][image15]

The rotation from A to B is equal to the transpose of the rotation from B to A. 
NOTE: Because rotation matrices are composed of orthogonal unit vectors (orthonormal) they have the following useful properties.

0. The transpose is equal to its inverse
1. The determinant is equal to +1 (assuming right-handed coordinate system)
2. Columns (and rows) are mutual orthogonal unit vectors i.e the magnitde of any column or row is equal to one and the dot product of any two columns or rows is equal to zero.
3. The columns define the basis vectors (i.e x,y,z axes) of the rotated frame relativee to the base frame. 

# Rotation Matrices in 3D

The same properties that apply in 2D also work for 2D.
![alt text][image151] 

projecting the basis vectore of 1 frame onto another 

Elementary rotations

![alt text][image152]

# Rotations in Sympy

Sympy is a full-featured computer algebra system (CAS) that allows for the construction and manipulation of matrices symbolically and can numerically evaluate them when necessary. 

Representing expressions symbolically has two major advantages

0. The visualization of equations proivde more insight into the system
1. Numerically. 
NOTE: Computers cannot perform floating point operations with infinite precision. 

Please see "SymPyRot.py" in this repository for an example implemention 

# Composition of Rotations
A sequence of rotation

Extrinsic
--- 

Euler Angle

orientation of any rigid boy wrt fixed frame acan be describe by three elementary rotations in a given sequence 

Properties

COnventions

0. Tait-Bryan vs Classic
1. Rotation order
2. Intrinsic (body fixed) vs Extrinsic (fixed axis) rotations

Tait-Bryan
---
each elementary rotatio is perfomred about a different Catesian axis 

Rotation order
---
NPTE none-communiative 

yz =/ zy

Extrinsic vs Intrinsic Rotation
---

Extrinsic rotations are performed about the fixed world reference frame

![alt text][image153]

y-z of fixed reference
NOTE: subsequent elementary rotations are pre-multiplied 

Intrinsic rotations are performed about the current reference frame

![alt text][image154]

NOTE: subsequent elementary rotations are post-multiplied

Drawbacks to euler angles
---

0. Numerical performance - When defining the orientation of a rigid body in 3D, we only need three generalized coordinates; however, rotation matrices contain nine elements. Not all 9 elements are independent and more memory is required to capture the orientation information.

1. Numerical stability -> numerical drift - rounding errors that occur with the repeated multiplication of rotation.
NOTE: overtime, the orthonormality conditions can be violated and the matrix is no longer a valid rotation matrix. 

2. Singularities of representation (gimbal lock) - occur when the second rotation in the sequence is such that the firat and third coordniates frames become aligned causing a loss of a degree of freedom. 

NOTE: ensure that the range of motion of the object of interest does not come close to a singularity

# Euler Angles from a Rotation Matrix

In cases involving Inverse Kinematics, we are given a composite rotation matrix and we need to find a set of Euler ANgles that would produce this rotation. 
NOTE: Although the specific solution depends on the choice of Euler angles, the basic procedure is the same. 

# Example 

Consider the extrinsic X-Y-Z rotation sequence. The composite rotation matrix is 

![alt text][image155]

Given the numerical values for rij, find the angles alpha, beta and gama.

Solution:

use various combinations or rij so that each angle can be individually isolated and solved explicitly. 
NOTE: To avoid sign ambiguity, DO NOT use inverse sine or cosine functions to solve for the angles.
NOTE: We use the atan2 function to avoid this ambiguity

![alt text][image156]


NOTE: when cos(beta) = 0 i.e beta = 90 degrees, atan2 is undefined and the system exhibits a singularity of representation. 

Please see rot2euler.py in this repository to get an example of how this is accomplished

Results of code as is 
---
alpha is  45.0000000000000 degrees
beta is   60.0000000000000 degrees
gamma is  30.0000000000000 degrees

# Translations 

Consider two reference frames A and B that have the same orientation but their origins are not coincident. 

![alt text][image157]


The position of point P relative to B is denonted by the vector B_r_P/B 

NOTE: The leading superscript describes the reference frame from which the vector r is being measured.

The following equation fully describe the position vector r relative to the B frame 

B_r_P/B = rBx x bx_hat + rBy x by_hat + rBz x bz_hat

We now describe P relative to A, A_r_P/A
NOTE: describing P relative to A is vector addition 

A_r_P/A = A_r_B/A + B_r_p/B

NOTE: The offset for a translation is calculated from the perspective of the new frame

# Homogeneous Transformations and their Inverse

Consider a more general case in which reference frames are both simultaneously rotated and translated with respect to each other. 

In the figure below we see point P, whose position is known relative to B and our goal is to again find A_r_P/A.


![alt text][image16]


This time our reference frames A and B do not have the same orientation. Therefore we must develop a new solution by combining the idea of translations and rotations.

The solution:

Express B_r_P/B in terms of Frame A by applying a rotation matrix between B and A. Next, add the distance between the origin of reference frame B relative to A

A_r_P/A = A/B_R x B_r_P/B + A_r_B/A

NOTE: The above equation is not easily handled bny computers in this form 

We will convert the above equation into matrix form using homogeneous coordinates. 

![alt text][image17]



The left side of the above equation is a 4x1 vector . The first three element are the x,y,z coordinate of point P expresend in the A frame.

The first term on the right side of the equation is a 4x4 matrix called the homogeneous transform .
It is composed of four sub-matrices.

0. (A/B)R is a 3x3 rotation matrix
1. A_r_B/A is a 3x1 vector and represents the origin of frame B relative to frame A 

The final term on the right hand side is a 4x1 vector where B_r_P/B is the location of P relative to B and is expressed in terms of the B frame.

![alt text][image18]


Multiplying the the matrices on the right brings us back to the original equation (the one that is difficult for the computer to interpert)


Homogenous transforms can be used to calculate the position of a robot arm's end effector.

# Example

Given the position of the end effector with respect to reference frame B, calculate its position with respet to reference frame A (attached to Link 1)

![alt text][image19] 


Please see the starter.py for an example on how to use python programming to solve this problem. 


Inverse operation (finding the transfrom B to A) 
---

Inverting the transform is a simple as inverting the 4x4 matrix. 

![alt text][image20]


# Composition of Homogenous Transforms




The composition of Homogeneous Transforms mirror the same logic as composition of rotations


![alt text][image21]

Assume the transform from frame C relative to frame B is already known. Also the transform from B to A is known. We can therefore express the vector r as the postion of P from the origin of C in frame A by first transforming it to the B frame 

B_r_P/B = (B/C)T x c_r_P/C 

and then transofrming to the A frame

A_r_P/A = (A/B)T B_r_P/B


Combining the two above equations yeilds


A_r_P/A = (A/B)T x (B/C)T x C_r_P/C = (A/C)T x C_r_P/C


# Denavit-Hartenberg Parameters

In order to use the homogenous transform to locate the end effector we must a reference frame to each link of the manipulator and write the transforms from the fixed base link to link 1, to link 2, all the way to the end eff. 

NOTE: Each transform would require six independent parameters to describe frame i relative to frame i-1
... 3 for position and 3 for orientation. 

THe DH convention proposed back in 1955 only requires 4 parameters to describe the position and orientation of neighboring reference frames. 

NOTE: There are 5 common versions of this method. It should always be noted which method is being used in order to limit confusion. In this respository we will be using number (2)

0. Waldron, KJ. A study of overconstrained linkage geometry by solution of closure equations, Part I: A method of study (1973). Mech. Mach. Theory 8(1):95-104.

1. Paul, R. (1982). Robot Manipulators: Mathematics, Programming and Control (MIT Press, Cambridge, MA)

2. Craig, JJ. (2005). Introduction to Robotics: Mechanics and Control, 3rd Ed (Pearson Education, Inc., NJ)

3. Khalil, W and Dombre, E. (2002). Modeling, Identification and Control of Robots (Taylor Francis, NY)

4. M. Spong and M. Vidyasagar, Robot Modeling and Control, Wiley, 2005

The DH Parameters:
---

<a href="https://www.codecogs.com/eqnedit.php?latex=\alpha&space;_{i-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\alpha&space;_{i-1}" title="\alpha _{i-1}" /></a>

The twist angle is defined as the angle between <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{Z}_{i-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{Z}_{i-1}" title="\hat{Z}_{i-1}" /></a> and <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{Z}_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{Z}_{i}" title="\hat{Z}_{i}" /></a> measured along <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{X}_{i-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{X}_{i-1}" title="\hat{X}_{i-1}" /></a>


<a href="https://www.codecogs.com/eqnedit.php?latex=a_{i-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?a_{i-1}" title="a_{i-1}" /></a>

The link length is the distance between <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{Z}_{i-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{Z}_{i-1}" title="\hat{Z}_{i-1}" /></a> and <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{Z}_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{Z}_{i}" title="\hat{Z}_{i}" /></a> measured along <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{X}_{i-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{X}_{i-1}" title="\hat{X}_{i-1}" /></a>, where <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{X}_{i-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{X}_{i-1}" title="\hat{X}_{i-1}" /></a> is perpendicular to both <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{Z}_{i-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{Z}_{i-1}" title="\hat{Z}_{i-1}" /></a> and <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{Z}_{i-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{Z}_{i-1}" title="\hat{Z}_{i-1}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=d_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?d_{i}" title="d_{i}" /></a>

The link offset is the signed distance from <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{X}_{i-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{X}_{i-1}" title="\hat{X}_{i-1}" /></a> to <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{X}_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{X}_{i}" title="\hat{X}_{i}" /></a> measured along <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{Z}_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{Z}_{i}" title="\hat{Z}_{i}" /></a> for prismatic joints. 

<a href="https://www.codecogs.com/eqnedit.php?latex=\theta_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta_{i}" title="\theta_{i}" /></a>

The joint angle is the angle between <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{X}_{i-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{X}_{i-1}" title="\hat{X}_{i-1}" /></a> to <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{X}_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{X}_{i}" title="\hat{X}_{i}" /></a> measured along <a href="https://www.codecogs.com/eqnedit.php?latex=\hat{Z}_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\hat{Z}_{i}" title="\hat{Z}_{i}" /></a> in a right hand sense for revolute joints.

![alt text][image22]

In general
---
For an n-degree of freedom manipulator there will be n-joints (1,2,...,n) and because every joint connects two links, there are n+1 links (0,1,...,n).

NOTE: The fixed base link is link 0 and the index, i, increases by 1 towards the end effector. 
NOTE: There is no requirement for the origin of the frame to be physically on the link, rather it needs to move rigidly with the link. 

The Z-axis os the reference frame is aligned with:
0. The axis of rotation for revolute joints 
1. The direction of motion for prismatic joints.

The homogeneous transform from frame i-1 to frame i is constructed as a sequence of four basic transformations, two rotations and two translations as follows:

<a href="https://www.codecogs.com/eqnedit.php?latex=_{i}^{i-1}\textrm{T}=&space;R_{x}(\alpha_{i-1}&space;)&space;\times&space;D_{x}(a_{i-1})&space;\times&space;R_{z}(\theta{i})&space;\times&space;D_{z}(d_{i})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?_{i}^{i-1}\textrm{T}=&space;R_{x}(\alpha_{i-1}&space;)&space;\times&space;D_{x}(a_{i-1})&space;\times&space;R_{z}(\theta{i})&space;\times&space;D_{z}(d_{i})" title="_{i}^{i-1}\textrm{T}= R_{x}(\alpha_{i-1} ) \times D_{x}(a_{i-1}) \times R_{z}(\theta{i}) \times D_{z}(d_{i})" /></a>

 A question you should always ask yourself is: "Does the transform move the reference frame in link i-1 to be exactly coincident with the reference frame in link i?" If the answer is yes, that is a good sign! 


# DH Parameter Assignment Algorithm 

In this section we will focus on the assignment algorithm that can be applied when dealing with a new manipulator.

Label all joints from {1, 2, … , n}.

Label all links from {0, 1, …, n} starting with the fixed base link as 0.

Draw lines through all joints, defining the joint axes.

Assign the Z-axis of each frame to point along its joint axis.

Identify the common normal between each frame \hat{Z}_{i-1} 
Z
^
  
i−1
​	  and frame \hat{Z}_{i} 
Z
^
  
i
​	  .

The endpoints of "intermediate links" (i.e., not the base link or the end effector) are associated with two joint axes, {i} and {i+1}. For i from 1 to n-1, assign the \hat{X}_{i} 
X
^
  
i
​	  to be …

For skew axes, along the normal between \hat{Z}_{i} 
Z
^
  
i
​	  and \hat{Z}_{i+1} 
Z
^
  
i+1
​	  and pointing from {i} to {i+1}.
For intersecting axes, normal to the plane containing \hat{Z}_{i} 
Z
^
  
i
​	  and \hat{Z}_{i+1} 
Z
^
  
i+1
​	 .

For parallel or coincident axes, the assignment is arbitrary; look for ways to make other DH parameters equal to zero.

For the base link, always choose frame {0} to be coincident with frame {1} when the first joint variable ( {\theta}_{1}θ 
1
​	  or {d}_{1}d 
1
​	 ) is equal to zero. This will guarantee that {\alpha}_{0}α 
0
​	  = {a}_{0}a 
0
​	  = 0, and, if joint 1 is a revolute, {d}_{1}d 
1
​	  = 0. If joint 1 is prismatic, then {\theta}_{1}θ 
1
​	 = 0.

For the end effector frame, if joint n is revolute, choose {X}_{n}X 
n
​	  to be in the direction of {X}_{n-1}X 
n−1
​	  when {\theta}_{n}θ 
n
​	  = 0 and the origin of frame {n} such that {d}_{n}d 
n
​	  = 0.



# The 8 DH Steps

Step 1 

Label all joints from 1 to n

Step 2 

Label all links from 0 to n
NOTE: links do nto have to fixed s

Step 3

Draw a line defining all joint axes

Step 4 

Define the common normals between joint axes
NOTE: We are looking for geometrical relationships between joints (i.e parallel, coincident, etc.) 
NOTE: Look for ways to minimize non-zero DH paramaters.

Step 5

Assign the Z-axis of frame i to point along the i-th joint axis. Decide if positive Z-axis is up or down. 

Step 6

Define positive x axis for links between base and end-effector. The x axis should point along the common normal from Zi-1 to Zi
NOTE: x needs to b perpendicular to z

Step 7
Assign x axis of link 0  
NOTE: The number of non-zero DH parameters can be minimized by chosing x0 to be coincident with x1 when joint1 is 0 and always choising z0 ro be coincident with z1

Step 8 
Assigning the x-axis for link n, the last link. 
NOTE: Alwayss chose x_n to be in the same direction as xn-1 when the last joint has a 0 angle or 0 displacement. x4 is parallel is x3

# DH Parameter Table 

Each row is the transfrom from link i-1 to link i 


# Forward Kinematics

In Forward Kinematics we know all the joint variable (i.e the generalized coordinates associated with revolut and prismatic jointss)
Now we can calculate the pose (the position and orientation) of the end effector in 3D space. 

The problem is a composition of homogenous transforms. Starting at the base link we move link by link to the end effector while using the DH parameters to build each individual transform. 

The image below is the total transform between adjacent links.
![alt text][image24] 


![alt text][image23]

# Coding the Forward Kinematics for a SCARA Robot

Using the robot below we will write code to calulate the location of the pose of the robot

![alt text][image24]

Please see FK_example.py in this repository.

# Inverse Kinematics

# Example

