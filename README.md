vfclik
======

VFCLIK: Vector field based closed loop inverse kinematics controller.

It is a robot controller that moves a manipulator towards a goal while avoiding obstacles using a vector field approach. It is reactive and runs on real time; the current velocity given to the robot is calculated in every control cycle based on the current position of the end effector of the robot, the goal position and the obstacle positions.

Use vfclik.py to execute all the system. You may need other dependencies available in github.com/arcoslab/

