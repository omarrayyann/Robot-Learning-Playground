# Libraries Importing
import pybullet as p
import numpy as np
import time
import pybullet_data

# Setting up pybullet
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

# Loading Asset
p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])
targid = p.loadURDF("franka_panda/panda.urdf", [0,0,0],[0,0,0,1],useFixedBase=True)
object_of_focus = targid

jointid
for step in range(300):
    focus_position, _ = p.getBasePositionAndOrientation(targid)
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=focus_position)
    p.stepSimulation()
    time.sleep(0.01)
