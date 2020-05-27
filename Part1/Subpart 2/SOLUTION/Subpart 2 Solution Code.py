import pybullet as p
import time
import pybullet_data

physicsClient=p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,0)
planeID=p.loadURDF('plane.urdf')
ArmPos=[0,0,0]
ArmOrient=p.getQuaternionFromEuler([0,0,0])
armID=p.loadURDF('r2d2.urdf',ArmPos,ArmOrient,useFixedBase=1)
p.setJointMotorControlArray(armID,[0,1],controlMode=p.VELOCITY_CONTROL,targetVelocities=[1,1])
while(10000):
    p.stepSimulation()
    time.sleep(1/240)
print('Simulation Ended!!')
p.disconnect()
