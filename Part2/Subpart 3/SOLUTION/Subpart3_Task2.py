import pybullet as p
import time
import pybullet_data

physicsClient=p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeID=p.loadURDF("plane.urdf")
start_pos=[0,0,3]
start_orient=p.getQuaternionFromEuler([0,0,0])
cubeIDs=[]

for i in range(6):
    cubeIDs.append(p.loadURDF("cube_small.urdf",start_pos,start_orient,useFixedBase=True))
    start_pos[1]+=0.11                                        #kept 1.11 so that the spheres can also be kept at such a distance

sphere_startpos=[0,0,1]
sphereIDs=[]

for i in range(6):
    sphereIDs.append(p.loadURDF("sphere.urdf",sphere_startpos,start_orient))
    p.changeDynamics(sphereIDs[i],-1,restitution=0.999)        #using ChangeDynamics to make the restitution of the ball = 1    
    sphere_startpos[1]+=0.11                                   #the spheres were kept a little bit apart so that collisions can occur as two body collisions

for i in range(6):
    p.createConstraint(cubeIDs[i],-1,sphereIDs[i],-1,p.JOINT_POINT2POINT,[0,0,1],[0,0,0],[0,0,2])     #constraints between the cube and the spheres

sphere_pos,sphere_orient=p.getBasePositionAndOrientation(sphereIDs[0])

while sphere_pos[2]<1.3:
    p.applyExternalForce(sphereIDs[0],-1,[0,-150,0],[0,0,0],p.LINK_FRAME)                   #moving the initial sphere upto a particular height using external force
    p.stepSimulation()
    sphere_pos,sphere_orient=p.getBasePositionAndOrientation(sphereIDs[0])
    time.sleep(1/100)
while True:
    p.stepSimulation()
    time.sleep(1/100)
p.disconnect()