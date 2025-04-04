import numpy as np
from urbasic import Joint6D, ISCoin
from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics
from time import sleep
from math import radians


class PenHolder:
    def __init__(
        self,
        iscoin: ISCoin,
        mapping: dict[int, tuple[int]],
        xPos=0.265 - 0.097,
        yPos=0.305 + 0.003,
    ):
        self.pen_positions = []
        self.heldPen = None
        z = 0.35
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
        kine_poses = [[p[0], p[1], p[2], -1, -0.5, 0, 0] for p in points]
        return self.multikine.inverse_optimal(kine_poses).trajectory

    def _findPen(self, pen):
        # Find the pen position based on the pen number
        for key, value in self.mapping.items():
            if value == pen:
                print(f"Pen {key} found in mapping.")
                return key
        raise ValueError(f"Pen {pen} not found in mapping.")

    def pickPen(self, pen_map):
        points = []
        pen = self._findPen(pen_map)

        # TODO: Change that with if not none -> drop it

        if self.heldPen is not None:
            self.dropPen()

        points.append(self.pen_positions[pen])
        points.append([self.pen_positions[pen][0], self.pen_positions[pen][1], 0.25])

        points.append([self.pen_positions[pen][0], self.pen_positions[pen][1], 0.18])
        points.append([self.pen_positions[pen][0], self.pen_positions[pen][1], 0.25])
        # close
        points.append(self.pen_positions[pen])
        self.heldPen = pen_map
        traj = self._getTrjectory(points)
        self._moveToPen(traj, False)

    def dropPen(self):
        if self.heldPen is None:
            raise ValueError("No pen held, cannot drop.")
        points = []
        pen = self._findPen(self.heldPen)
        if self.heldPen is not None:
            points.append(self.pen_positions[pen])
            points.append(
                [self.pen_positions[pen][0], self.pen_positions[pen][1], 0.25]
            )
            points.append([self.pen_positions[pen][0], self.pen_positions[pen][1], 0.2])
            points.append(
                [self.pen_positions[pen][0], self.pen_positions[pen][1], 0.25]
            )
            # open
            points.append(self.pen_positions[pen])
            self.heldPen = None
        else:
            raise ValueError(f"Pen {pen} not held, cannot drop.")
        traj = self._getTrjectory(points)
        self._moveToPen(traj, True)

    def switchPen(self, pen_mapped):
        pen = self._findPen(pen_mapped)
        self.dropPen()
        self.heldPen = None
        self.pickPen(pen_mapped)

    def _moveToPen(self, angles, openGripper=False):
        for i, a in enumerate(angles):
            jo = Joint6D.createFromRadList(a)
            self.iscoin.robot_control.movej(jo, a=radians(70), v=radians(120))
            if i % 5 == 2:
                if openGripper:
                    self.iscoin.gripper.open()
                    sleep(1.5)
                else:
                    self.iscoin.gripper.close()
                    sleep(1.5)


if __name__ == "__main__":

    def interpolate(start, end, nbr_points=5):
        start = np.array(start)
        end = np.array(end)
        step = (end - start) / (nbr_points + 1)

        points = [start.tolist()]
        for i in range(nbr_points):
            points.append((start + (step * (i + 1))).tolist())

        points.append(end.tolist())

        return points

    def getCubeLines(corners):
        topLeft = np.array(corners[0])
        topRight = np.array(corners[1])
        cubeCenterHigh = (topLeft + topRight) / 2
        cubeCenterHigh[2] += 0.15
        cubeCenterHigh = [-0.05, 0.25, 0.40]
        lines = []
        for i in range(len(corners)):
            start = corners[i]
            end = corners[(i + 1) % 4]
            traj = [cubeCenterHigh, [*start[:2], start[2] + 0.1]]
            traj += interpolate(start, end)
            traj.append([*end[:2], end[2] + 0.1])
            traj.append(cubeCenterHigh)
            lines.append(traj)
        return lines

    res = interpolate([0], [6])
    colors = {
        "green": (0, 255, 0),
        "red": (255, 0, 0),
        "blue": (0, 0, 255),
        "pink": (255, 105, 180),
        "cyan": (0, 255, 255),
        "white": (255, 255, 255),
        "black": (0, 0, 0),
        "purple": (128, 0, 128),
    }

    x = (-0.15, -0.25)
    y = (0.15, 0.25)
    z = 0.175

    corners = [
        [x[0], y[0], z],
        [x[1], y[0], z],
        [x[1], y[1], z],
        [x[0], y[1], z],
    ]

    lines = getCubeLines(corners)

    lines_trajectories = []

    kine = URKinematics("ur3e_pen_gripper")
    multikine = MultiURKinematics(kine)

    for l in lines:
        ee_poses = [[*p, -1, 0, 0, 0] for p in l]

        traj = multikine.inverse_optimal(ee_poses)
        lines_trajectories.append(traj)

    iscoin = ISCoin(host="10.30.5.158", opened_gripper_size_mm=40)

    test = PenHolder(
        iscoin,
        {
            0: colors["green"],  # green
            1: colors["red"],  # red
            2: colors["blue"],  # blue
            3: colors["pink"],  # pink
            4: colors["cyan"],  # cyan
            5: colors["white"],  # white
            6: colors["black"],  # black
            7: colors["purple"],
        },
    )

    speed = radians(120)
    acc = radians(70)

    def executeTraj(traj):
        for a in traj:
            iscoin.robot_control.movej(Joint6D.createFromRadList(a), a=speed, v=acc)

    test.pickPen(colors["red"])

    executeTraj(lines_trajectories[0])

    test.switchPen(colors["white"])

    executeTraj(lines_trajectories[1])

    test.switchPen(colors["green"])

    executeTraj(lines_trajectories[2])

    test.switchPen(colors["cyan"])

    executeTraj(lines_trajectories[3])

    test.dropPen()

    # TODO: Check if necessary

    # test.switchPen((0,255, 255))
    # sleep(2)
    # test.dropPen((255,255, 255))
    # sleep(2)
    # test.switchPen((255,0, 0))
    # sleep(2)
    # test.dropPen((255,0, 0))