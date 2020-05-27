import pybullet as p
import pybullet_data
import time

physicsClient=p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeID=p.loadURDF("plane.urdf")
#these are the pre required conditions for the task.
ramp=p.loadURDF("wedge.urdf")
p.setGravity(0, 0, -10)
p.changeDynamics(ramp,-1,lateralFriction=0.5)

huskypos = [2, 0, 0.1]
husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])

num_joints=p.getNumJoints(husky)
print("Number of joints in husky is : " + str(num_joints))

for i in range(num_joints):
	print("Info for joint " + str(i) + " is : ")
	print(p.getJointInfo(husky,i))                                    #printing joint info in the beginning of the code
print("Do you want Velocity Control or Torque Control?")
num=input()
num=int(num)                                                          #for velocity control input 0, for torque control input 1 

def Torque_control():
	optimal_torque_value = -270
	forces_tc=[]
	for i in range(4):
		forces_tc.append(optimal_torque_value)
	p.setJointMotorControlArray(husky,[2,3,4,5],controlMode=p.TORQUE_CONTROL,forces=forces_tc)


def Velocity_control():
	maxForce = -10
	optimal_velocity_value = 30
	forces_vc=[]
	velocity_vc=[]
	for i in range(0,4):
		forces_vc.append(maxForce)
		velocity_vc.append(optimal_velocity_value)
	p.setJointMotorControlArray(husky,[2,3,4,5],controlMode=p.VELOCITY_CONTROL,targetVelocities=velocity_vc,forces=forces_vc)

count=0

while (1):
	time.sleep(.01)
	p.stepSimulation()
	if num==0:
		Velocity_control()
	elif num==1:
		Torque_control()
	count=count+1
	if count%100==0:
		print("The base link info is : ")
		print(p.getLinkState(husky,0,computeLinkVelocity=1,computeForwardKinematics=1))                 #printing link state and base velocity every 100th iteration
		print("The base link velocity is : ")
		print(p.getBaseVelocity(husky))
p.disconnect()
