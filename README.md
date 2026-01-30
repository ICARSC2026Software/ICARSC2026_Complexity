# Complexity Movement Evaluation

Code for evaluation of the complexity of a given movement.

The complexity score is based on a weighted sum of seven individual metrics derived from the initial and final poses of a movement. 
The seven individual metrics used are:
- The coordinates of the target position ($x_f$, $y_f$, $z_f$);
- The distance between the final and initial position of the movement ($\Delta_{d}$);
- The reorientation of the end-effector from the initial to the final position ($\Delta_{\phi}$, $\Delta_{\theta}$ and $\Delta_{\psi}$)

## computeComplexity
If the data relative to the initial and final position is available.
It has as input the following arguments:
- distance - distance between the final and initial point
- xf - final X position of the target position in relation to the robot reference frame
- yf - final Y position of the target position in relation to the robot reference frame
- zf - final Z position of the target position in relation to the robot reference frame
- droll - reorientation in roll(Z) between the final and initial pose of the end-effector;
- dpitch - reorientation in pitch(Y) between the final and initial pose of the end-effector;
- dyaw - reorientation in yaw(X) between the final and initial pose of the end-effector
- lengthArm - length between the shoulder and the palm of the hand
- filename (optional) - give a filename if you want to save the data to a file

## computeComplexityTraj
If the goal is to compute the complexity of a trajectory already computed.
It has as input the following arguments:
- Denavit-Hartenberg Parameters (Modified Convention) in form of struct
	- DH.d = 1xnJoints
	- DH.a = 1xnJoints
	- DH.alpha = alpha
	- Trajectory Filename
	- rpyArm (optional) - RPY (ZYX) of the arm  - if none given it will be considered [0 0 0]
    - rpyArmLeft (optional) - In case of bimanual movements, is the RPY (ZPY) of the left arm - if none given it will be considered [0 0 0]
	
## Example 
An example of how both functions can be used.


