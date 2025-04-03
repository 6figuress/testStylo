import time
from simulator import Simulator
import pybullet as p
import numpy as np
simulator = Simulator(True)




# Get updated position of the pen holder
pen_support_position, _ = p.getBasePositionAndOrientation(simulator.robot_id)
print("New Pen Support Position:", pen_support_position)
local_point = np.array([0.27-0.055, 0.3-0.055, 0.18])# after 24 23


# Calculate the ground point directly below the local point
ground_point = np.array([local_point[0], local_point[1], 100])

# Draw a perpendicular line from the ground to the local point
p.addUserDebugLine(
    lineFromXYZ=ground_point,  # Start point at the ground
    lineToXYZ=local_point,  # End point at the local point
    lineColorRGB=[0, 1, 0],  # Green color for the line
    lineWidth=2,
    lifeTime=0  # Keeps the line visible indefinitely
)

# Adjust the camera to ensure everything is in view
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=pen_support_position)


# Keep the simulation running
while p.isConnected():
    simulator.moveToAngles([-1.46499658, -2.86547637,  0.89621556,  0.39650053, -1.5710876 ,
        -3.03632379])



angles0 =  [-1.66909659, -2.93423128,  1.40736115,  3.09732533,  1.5717696 ,
        -0.09966348]

anglesTeststylo = [-2.08425879, -1.73630238, -1.44088173, -1.53622222,  1.57123637,
        -0.51445681]