from ur_ikfast.ur_kinematics import URKinematics


kine = URKinematics("ur3e_pen_gripper")

x = 0.27
y = 0.3
z = 0.2

angles = kine.inverse([x, y, z, -1, 0, 0, 0], all_solutions = True)

import ipdb
ipdb.set_trace()