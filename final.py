from urbasic import Joint6D,ISCoin
from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics
from time import sleep
from math import radians

class PenHolder():
    def __init__(self, iscoin:ISCoin, mapping:dict[int, tuple[int]], xPos = 0.265, yPos=0.305):
        self.pen_positions = []
        self.heldPen = None
        z = 0.3
        # Generate positions for the pens in a 2x4 grid
        for row in range(2):
            y = yPos - row * 0.06
            for col in range(4):
                x = xPos - col * 0.053
                self.pen_positions.append([x, y, z])
        self.mapping = mapping
        self.iscoin = iscoin
        self.iscoin.gripper.activate()
        sleep(3)
        self.iscoin.gripper.open()
        sleep(3)
        kine = URKinematics("ur3e_pen_gripper")
        self.multikine = MultiURKinematics(kine)
    def _getTrjectory(self, points):
        kine_poses = [[p[0], p[1], p[2], -1, -0.5, 0, 0]for p in points]
        import ipdb;ipdb.set_trace()
        return self.multikine.inverse_optimal(kine_poses).trajectory
    
    def _findPen(self, pen):
        # Find the pen position based on the pen number
        print(self.mapping)
        for key, value in self.mapping.items():
            if value == pen:
                print(f"Pen {key} found in mapping.")
                return key
        else:
            raise ValueError(f"Pen {pen} not found in mapping.")
        
    def pickPen(self, pen_map):
        points = []
        pen = self._findPen(pen_map)
        if self.heldPen is None: 
            points.append(self.pen_positions[pen])
            points.append([self.pen_positions[pen][0], self.pen_positions[pen][1], 0.25])

            points.append([self.pen_positions[pen][0], self.pen_positions[pen][1], 0.18])
            points.append([self.pen_positions[pen][0], self.pen_positions[pen][1], 0.25])
            # close
            points.append(self.pen_positions[pen])  
            self.heldPen = pen_map
            print(points)
        else: 
            self.switchPen(pen)
        traj = self._getTrjectory(points)
        self._moveToPen(traj, False)



        
    def dropPen(self, pen):
        points = []
        pen = self._findPen(pen)
        if self.heldPen is not None: 
            points.append(self.pen_positions[pen])
            points.append([self.pen_positions[pen][0], self.pen_positions[pen][1], 0.25])
            points.append([self.pen_positions[pen][0], self.pen_positions[pen][1], 0.2])
            points.append([self.pen_positions[pen][0], self.pen_positions[pen][1], 0.25])
            # open
            points.append(self.pen_positions[pen])  
            self.heldPen = None
        else: 
            raise ValueError(f"Pen {pen} not held, cannot drop.")
        traj = self._getTrjectory(points)
        self._moveToPen(traj, True)
      

    def switchPen(self, pen_mapped):
        pen = self._findPen(pen_mapped)
        print("held pen", self.heldPen)
        self.dropPen(self.heldPen)
        self.heldPen = None
        self.pickPen(pen_mapped)
    
    def _moveToPen(self, angles, openGripper = False):
        print("N pos : ", len(angles))
        for i, a in enumerate(angles):
            jo = Joint6D.createFromRadList(a)
            #print(f'Joints are at {"iscoin.robot_control.get_actual_joint_positions()"} - going to {jo}')
            self.iscoin.robot_control.movej(jo, a = radians(10), v = radians(10))
            print(f"I'm at pos : {i}")
            if i % 5 == 2:
                print(openGripper)
                if openGripper: 
                    print("open2")
                    self.iscoin.gripper.open()
                    sleep(1.5)
                else:
                    print("close")
                    self.iscoin.gripper.close()
                    sleep(1.5)
            

if __name__ == "__main__":
    test = PenHolder(ISCoin(host="10.30.5.158", opened_gripper_size_mm=40), {
                        0: (0,255, 0), # green
                        1: (255,0, 0), # red
                        2: (0,0, 255), # blue
                        3: (255,105,180), # pink
                        4: (0,255, 255), # cyan
                        5: (255,255, 255), # white
                        6: (0,0, 0), # black
                        })
    test.pickPen(((0,255, 0)))
    sleep(2)
    # test.switchPen((0,255, 255))
    # sleep(2)
    # test.dropPen((255,255, 255))
    # sleep(2)
    # test.switchPen((255,0, 0))
    # sleep(2)
    # test.dropPen((255,0, 0))