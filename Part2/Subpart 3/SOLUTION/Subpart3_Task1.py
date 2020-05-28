import pybullet as p
import pybullet_data
import time
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
extraVel = 0           #to count the number of times 'a' has been pressed and increase the speed of the motors accordingly

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

p.getContactPoints(car)
p.disconnect()