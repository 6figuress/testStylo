from time import sleep
from simulator import Simulator
import pybullet as p
import numpy as np
from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics
from math import radians
from urbasic import Joint6D,ISCoin

kine = URKinematics("ur3e_pen_gripper")
multikine = MultiURKinematics(kine)
# Initialize simulator
# simulator = Simulator(gui=True)

iscoin = ISCoin(host="10.30.5.158", opened_gripper_size_mm=40)

safePositionStart = [0.2, 0.15, 0.4]
anglesafe = kine.inverse([safePositionStart[0], safePositionStart[1], safePositionStart[2], -1, 0, 0, 0], all_solutions = True)[1]
print("Safe Position Angles:", anglesafe)
safePositionEnd = []
# Define positions of eight pens
z = 0.25
pen_positions = []
angles = []
# Generate positions for the pens in a 2x4 grid
for row in range(2):
    y = 0.305 - row * 0.06
    for col in range(4):
        x = 0.265 - col * 0.053
        pen_positions.append([x, y, z])
# Move the robot to each pen position to pick them up

points = []

for pos in pen_positions:
  


    # # Calculate the ground point directly below the local point
    # ground_point = np.array([pos[0], pos[1], 100])

    # # Draw a perpendicular line from the ground to the local point
    # p.addUserDebugLine(
    #     lineFromXYZ=ground_point,  # Start point at the ground
    #     lineToXYZ=pos,  # End point at the local point
    #     lineColorRGB=[0, 1, 0],  # Green color for the line
    #     lineWidth=2,
    #     lifeTime=0  # Keeps the line visible indefinitely
    # ) 
    # p.addUserDebugLine(
    #     lineFromXYZ=ground_point,  # Start point at the ground
    #     lineToXYZ=safePositionStart,  # End point at the local point
    #     lineColorRGB=[0, 1, 0],  # Green color for the line
    #     lineWidth=2,
    #     lifeTime=0  # Keeps the line visible indefinitely
    # )

    points.append([pos[0],pos[1],z])
    points.append([pos[0],pos[1],0.18])
    # close
    print("Pen Position:", pos)
    points.append([pos[0],pos[1],z])
    points.append([pos[0],pos[1],0.18])
    #open
    points.append([pos[0],pos[1],z])




kine_poses = [[p[0], p[1], p[2], -1, -0.5, 0, 0]for p in points]


angles = multikine.inverse_optimal(kine_poses).trajectory

# for i in angles:
#     for j in i:
#         if simulator.isValidConfiguration(j):
#             print("here")
#             safeAngles.append(j)
#             break
print("Safe Angles:", angles)
# Move the robot to each calculated position and perform the simulation step
# for angle in angles:
#     # time.sleep(1)  # Small delay to allow for visualization
#     time.sleep(0.5)
#     print("Moving to angle:", angle)
#     # simulator.moveToAngles(anglesafe)
#     simulator.moveToAngles(angle)  # Move to the specified angles

#     print("Reached angle:", angle)
#     # simulator.moveToAngles(anglesafe)  # Move back to the safe position

#     # Allow some time for the robot to reach the position before moving to the next

# # Keep the simulation running
# while p.isConnected():
#     p.stepSimulation()  # Continuously step through the simulation
#     time.sleep(0.01)  # Small delay to prevent high CPU usage
# jo = Joint6D.createFromRadList([-1.97275066, -1.9201802 , -1.20432866, -1.58667171,  1.5701735 ,
#         -0.40265095])
# 	#print(f'Joints are at {"iscoin.robot_control.get_actual_joint_positions()"} - going to {jo}')
# iscoin.robot_control.movej(jo, a = radians(10), v = radians(10))
iscoin.gripper.activate()
sleep(3)
iscoin.gripper.open()
sleep(3)
for i, a in enumerate(angles):
    jo = Joint6D.createFromRadList(a)
    #print(f'Joints are at {"iscoin.robot_control.get_actual_joint_positions()"} - going to {jo}')
    iscoin.robot_control.movej(jo, a = radians(15), v = radians(15))
    if i % 5 == 1:
        print("close")
        iscoin.gripper.close()
        sleep(1.5)
    elif i % 5 == 3:
        print("open")
        iscoin.gripper.open()
        sleep(1.5)