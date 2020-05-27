import pybullet as p
import time
import pybullet_data

def fib(i):
    if i==1:
        return 1
    elif i==2:
        return 1
    else:
        return fib(i-1)+fib(i-2)

x=1
PhysicsClient=p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p.loadURDF('plane.urdf')
startpos=[0,0,5]
startorientation=p.getQuaternionFromEuler([0,0,0])
sphereID=p.loadURDF('rainfall.urdf',startpos,startorientation)
sphereIDs=[sphereID]
while(True):                  
    p.stepSimulation()
    spherepos,sphereorient=p.getBasePositionAndOrientation(sphereIDs[0])
    if spherepos[2]<0.1:
        x=x+1
        startpos=[0,0,5]                                  
        for i in range(len(sphereIDs)):
            p.removeBody(sphereIDs[i])                  #removing the raindrops which have fallen
        sphereIDs.clear()
        for i in range(fib(x)):                
            sphereIDs.append(p.loadURDF('rainfall.urdf',startpos,startorientation))
            startpos[1]=startpos[1]+0.5                 #updating the y-coordinate for each raindrop so that all of them are equally spaced along x-axis
    time.sleep(1/240)
p.disconnect()