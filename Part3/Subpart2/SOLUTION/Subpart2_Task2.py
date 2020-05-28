#IMPORTANT NOTE - Please download the 2R_planar_robot URDF file which we have provided in the google drive folder and run our simulation on that URDF file as we have made some changes 
                  #to the URDF in order to facilitate faster simulation of our robot


import pybullet as p
import pybullet_data
import os
import time
import math
import numpy as np

file_name = "2R_planar_robot.urdf" #the URDF file which we have used is different from the one given as we have changed the joint velocities to facilitate faster simulation
p.connect(p.GUI)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF(file_name)
p.resetBasePositionAndOrientation(robot, [0, 0, 0.05], [0, 0, 0, 0.707])

p.setGravity(0,0,-10)

'''
Note:
*these are the two end point of the line segment you want to 
 draw,so after finding the line segment you want to trace change 
 the two point below.
*the function addUserDebugLine just draws the line segment joining
point_A and point_B 
'''

point_A = [0,0,2.1] #endpoints of the line that our end-effector will trace in this simulation
point_B = [0,2.1,0]
p.addUserDebugLine(point_A,point_B,[1,0,0],2)

l1 = 1 #update the length of link-1,from the urdf
l2 = 1 #updare the length of link-2,from the urdf

def Forward_kinematics(angle_1,angle_2):

	'''
	This function should do the necessary trignometric 
	calculations using angle_1,angle_2,l1,l2 and resturn the [0,y,z] as the robot
	works space is the y-z plane
	'''
	angle_1=np.pi/2-angle_1
	angle_2=angle_2
	y=l1*math.cos(angle_1) + l2*math.cos(angle_1+angle_2) # calculate y
	z=l1*math.sin(angle_1) + l2*math.sin(angle_1+angle_2)# calculate z
	return [0,y,z]

def Inverse_kinematics(target,num):
	'''
	This function should do the necessary trignometric 
	calculations using y ,z,l1,l2 and return angle_1 and angle_2 to 
	reach a given target of the form [0,y,z],as the robot 
	works space is th y-z plane.
	'''
	y=target[1]
	z=target[2]
	angle_2=math.acos((z**2+y**2-l1**2-l2**2)/2*l1*l2)#calculate angle_2
	a=l1+l2*(math.cos(angle_2)+math.sin(angle_2))
	b=l1+l2*(math.cos(angle_2)-math.sin(angle_2))
	denom=math.sqrt(a**2+b**2)
	if num==0:
		sum=math.asin(2/denom)
	elif num==1:
		sum=math.asin(-2/denom)+np.pi
	if b==0:
		phi=np.pi/2
	else:
		phi=math.atan(a/b)
	angle_1=sum-phi#calculate angle_1
	print("sum==" + str(sum))
	print("phi==" + str(phi))
	print("denom==" + str(denom))
	angle_1=-angle_1
	angle_2=-angle_2
	return angle_1,angle_2

'''
Write the equation of the line you are going to follow:- y+z=2.1
Example, 
*it might be of the for z = m*y + c, so in the
 while loop below increament y = y + dy and find new z
 and give the target ie [0,y,z] to the Inverse_Kinematics 
 function.
*so trace the line from point_A to point_B and reset position 
 to point_A and continue the same process infinitely.  
'''
count=0  #to keep track as to whether the end effector is going up or going down
angle_1=0
angle_2=0
ee_coord=[0,0,2.1]

while(True):
	if count==0:
		ee_coord[2]-=0.0001
	
	elif count==1:
		ee_coord[2]+=0.0001
	
	if ee_coord[2]<0.05:
		count=1
	elif ee_coord[2]>2.09:
		count=0

	ee_coord[1]=2.1-ee_coord[2]
	
	[angle_1,angle_2]  = p.calculateInverseKinematics(robot,2,ee_coord)
	
	p.setJointMotorControl2(bodyIndex=robot,
                            jointIndex=0,
                            controlMode =p.POSITION_CONTROL,
                            targetPosition=angle_1,
                            force=10000)

	p.setJointMotorControl2(bodyIndex=robot,
                            jointIndex=1,
                            controlMode =p.POSITION_CONTROL,
                            targetPosition=angle_2,
                            force=10000)
	p.stepSimulation()
	
p.disconnect()