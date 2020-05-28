import math
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data

p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

huskypos = [0, 0, 0.1]
target_block=p.loadURDF("block0.urdf", -3, 0, 0)

husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])
p.createConstraint(husky, -1, -1, -1, p.JOINT_POINT2POINT, [0,1,0], [0,0,0], [0,0,0])

#Setting Constants
width = 512
height = 512
fov = 90
aspect = width / height
near = 0.6
far = 10

maxForce = 200 #Newton.m
camera_pointing_to=[-10, 0, 0]


def turn(speed):
    baseSpeed = 100
    targetVel_R = baseSpeed + speed
    targetVel_L = baseSpeed - speed
    for joint in range(1, 3):
        p.setJointMotorControl2(husky, 2 * joint, p.VELOCITY_CONTROL, targetVelocity = targetVel_R, force = maxForce)
    for joint in range(1, 3):
        p.setJointMotorControl2(husky, 2 * joint + 1, p.VELOCITY_CONTROL, targetVelocity = targetVel_L, force = maxForce)
    p.stepSimulation()

'''
tune the kp and kd from experiments, 
a hint set kd = 10*kp
'''
Kp = 4
Kd = 45
last_error = 0
PID_CONTROL = False


while (True):
    keys = p.getKeyboardEvents()
    if (PID_CONTROL):
    	# 1. Get the image feed, as in subpart-1.
        basepos,baseorient = p.getBasePositionAndOrientation(husky)
        baseorientation = p.getEulerFromQuaternion(baseorient)
        dis = 2
        x = basepos[0] + dis * math.cos(baseorientation[2])
        y = basepos[1] + dis * math.sin(baseorientation[2])
        upvector=[]
        for i in range(3):
            upvector.append(basepos[i])
        upvector[2] = 1
        view_matrix = p.computeViewMatrix(basepos, [x, y, basepos[2]], upvector)
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
        images = p.getCameraImage(width,
                        height,
                        view_matrix,
                        projection_matrix,
                        shadow=True,
                        renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
        # cv2.imshow('rgb', rgb_opengl)
        # cv2.waitKey(1)
        # hsv_lower = np.array([80,0,75])
        # hsv_upper = np.array([150,255,255])
        up = np.array([180, 10, 10])
        lw = np.array([50, 0, 0])

        # 2. Using the above limits, mask the green colour 
        #    and draw the area with maximuma contour and find 
        #    its moments.
        rgb_opengl = cv2.cvtColor(rgb_opengl.astype('float32'), cv2.COLOR_BGR2HSV)
        # cv2.imshow('mask', rgb_opengl)
        mask = cv2.inRange(rgb_opengl, lw, up)
        M = cv2.moments(mask)
        # print(M["m00"])

        # If M["m00"] becomes zero
        try: # If Block is visible
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(mask, (cX, cY), 5, (100, 100, 100), -1)
            # cv2.putText(mask, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.circle(mask, (mask.shape[0] // 2, mask.shape[1] // 2), 5, (150, 20, 20), -1)
        
            cv2.imshow('mask', mask)
            cv2.waitKey(1)
            # 3. Calculate the error, and apply pid control to find the
            #    optimal speed correction and call the turn function with
            #    that speed. 
            #    Apply pid law to calulate this value, use Kp and Kd variable above
            #    and tune em properly.
            error = cX - mask.shape[0]//2
            speed_correction = error * Kp + (error-last_error) * Kd
            turn(speed_correction)
            last_error=error # initialize accordingly
        except: #If Block is not visible
            cv2.imshow('mask', mask)
            cv2.waitKey(1)
            speed_correction = 115
            turn(speed_correction)
        
    for k, v in keys.items():
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN) and PID_CONTROL==False):
            targetVel = 2
            for joint in range(1,3):
                p.setJointMotorControl2(husky,2*joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
            for joint in range(1,3):
                p.setJointMotorControl2(husky,2*joint+1, p.VELOCITY_CONTROL,targetVelocity =-1*targetVel,force = maxForce)

            p.stepSimulation()
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)and PID_CONTROL==False):
            targetVel = 2
            for joint in range(1,3):
                p.setJointMotorControl2(husky,2* joint+1, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            for joint in range(1,3):
                p.setJointMotorControl2(husky,2* joint, p.VELOCITY_CONTROL,targetVelocity =-1*targetVel,force = maxForce)

            p.stepSimulation()
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)

            p.stepSimulation()
        if (k == ord('c') and (v & p.KEY_WAS_TRIGGERED)):
            print("\nPID Control-on")
            PID_CONTROL = True
        if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
            print("\nPID Control-off, back to manual")
            PID_CONTROL = False
p.disconnect()
