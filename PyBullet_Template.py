"""
Created on Sun Jun 10 18:28:23 2018 by Richard Bloemenkamp
Modified by Shaheer Ziya Haris Rashid
"""
import pybullet as p
import time
import numpy as np
    
# Open up GUI
p.connect(p.GUI)

# Create a plane
planeID = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, planeID)

# Create collision shape and corresponding visual shape
boxDim = [1, 0.5, 0.5]
c_boxID = p.createCollisionShape(p.GEOM_BOX, halfExtents=boxDim)
# Connect the collision shape and visual shape (-1 by default). Throw it in the real world.
p.createMultiBody(0, c_boxID, basePosition=[0, 0, 0])

# Create collision shape and corresponding visual shape
r = 1
c_sphereID = p.createCollisionShape(p.GEOM_SPHERE, radius=r)
# v_sphereID = p.createVisualShape(p.GEOM_SPHERE, rgbaColor=[1,0.5, 1,0.5], radius=r)
# Connect the collision shape and visual shape. Throw it in the real world.
p.createMultiBody(0, c_sphereID)

# Advance time in simulation
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)


p.disconnect()