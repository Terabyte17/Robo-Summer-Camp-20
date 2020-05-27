import pybullet as p
import time
import pybullet_data
import numpy as np

def g_magnitude(x,y):
    return np.sqrt(x**2 + y**2)

def g_update(x):
    if x<9.8:
        x+=0.2
    else:
        x=0
    return x

PhysicsClient=p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
g=[0,0,0]
p.setGravity(g[0],g[1],g[2])   #initial gravity=0
planeID=p.loadURDF('plane.urdf')
CubeStartPos=[2,2,1]
DabbaStartPos=[0,0,1]
StartOrientation=p.getQuaternionFromEuler([0,0,0])
boxID=p.loadURDF('sample.urdf',CubeStartPos,StartOrientation)
dabbaID=p.loadURDF('dabba.urdf',DabbaStartPos,StartOrientation)
while(True):
    p.stepSimulation()
    mag=g_magnitude(g[0],g[1])           #to calculate magnitude of the gravity
    new_mag=g_update(mag)                #to update gravity
    g[0]=new_mag/np.sqrt(2)
    g[1]=new_mag/np.sqrt(2)
    p.setGravity(g[0],g[1],g[2])
    time.sleep(1/240)
print("Simulation Ended!")
p.disconnect()