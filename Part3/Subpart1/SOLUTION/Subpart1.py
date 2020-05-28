#IMPORTANT POINTS- Please download the dabba.urdf file as well which has been provided in the google drive folder as it has been used in this simulation to check the working of the camera
                   #the dabba.urdf file has been loaded at [5,0,0] and the camera will capture the image of the box correspondingly
                   #to continue after getting an image, you need to click on the image window and then press q to continue with your next input

import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np
import math

p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
carpos = [0, 0, 0.1]

car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))
targetVel = 10  #rad/s
maxForce = 100 #Newton
extraVel = 0 #to count the number of times 'a' has been pressed and increase the speed of the motors accordingly

dabbapos=[5,0,0]
p.loadURDF("dabba.urdf",dabbapos,p.getQuaternionFromEuler([0,0,0]))   #this has been loaded to check whether the camera is capturing the correct thing

width = 512
height = 512

fov = 60
aspect = width / height
near = 0.8
far = 10
dis=2    #the target distance is changed using this - the target point for the computeViewMatrix will be dis units in front of the baseposition.
upvector=[]

while (1):
   
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = 3 + extraVel
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity = targetVel,force = maxForce)
           
            p.stepSimulation()
        
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
          
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = -3 - extraVel
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
        
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):         #for left turn 
            targetVel = 1 + extraVel
            for joint in range(2,5,2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = -1*targetVel,force = maxForce)
            for joint in range(3,6,2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)

            p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2,6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)

            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):      #for right turn
            targetVel = 1 + extraVel
            for joint in range(2,5,2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            for joint in range(3,6,2):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = -1*targetVel,force = maxForce)

            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2,6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)

            p.stepSimulation()

        if (k==ord('r') and (v & p.KEY_IS_DOWN)):                 #for rotating about it's axis of symmetry
            targetVel = 4 + extraVel
            for joint in range(2,5,2):
               p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            for joint in range(3,6,2):
               p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = -1*targetVel,force = maxForce)
            p.stepSimulation()
            
        if (k==ord('r') and (v & p.KEY_WAS_RELEASED)):
            targetVel=0
            for joint in range(2,6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = 0,force=maxForce)
            p.stepSimulation()

        if (k==ord('a') and (v & p.KEY_WAS_RELEASED)):           #for increasing the speed of the motor every time 'a' is pressed
            extraVel=extraVel+1
            print("Now the additional velocity of the wheels is " + str(extraVel))

        if (k==ord('c') and (v & p.KEY_WAS_RELEASED)):           #for capturing an image from the car's perspective
            basepos,baseorient=p.getBasePositionAndOrientation(car)
            baseorientation=p.getEulerFromQuaternion(baseorient)
            x=basepos[0] + dis * math.cos(baseorientation[2])         #calculating the coordinates for the target point we will pass in the computeViewMatrix
            y=basepos[1] + dis * math.sin(baseorientation[2])
            for i in range(3):
                upvector.append(basepos[i])
            upvector[2]=1
            view_matrix=p.computeViewMatrix(basepos, [x,y,basepos[2]], upvector)
            projection_matrix=p.computeProjectionMatrixFOV(fov, aspect, near, far)
            images = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
            cv2.imshow('rgb', rgb_opengl)    #showing the image
            print("You need to press q after going to the rgb window to continue....")
            if cv2.waitKey(0) & 0xff==ord('q'):                    #you need to press q to continue
                cv2.destroyAllWindows()
            upvector.clear()

p.getContactPoints(car)
p.disconnect()