import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])


robot = p.loadURDF("./FourStickedBody/urdf/FourStickedBody.urdf", startPos, startOrientation)

maxForce = 100
mode = p.TORQUE_CONTROL 


for i in range(12):
    p.setJointMotorControl2(robot, i, controlMode=mode, force=maxForce, targetPosition=0)


for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)


cubePos, cubeOrn = p.getBasePositionAndOrientation(robot)


print(cubePos,cubeOrn)


p.disconnect()