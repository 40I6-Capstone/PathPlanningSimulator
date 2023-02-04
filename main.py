from RobotTypes import *
from time import sleep
import numpy as np
from PathScheduler import PathScheduler

robotRad = 0.05 # Robot radius in meters
dt = 0.02 # Simulation timestep
time = 0.0 # Current Time in simulation
simTime = 10 # Total Simulation time

r1 = IdealRobot.IdealRobot(robotRad)
slowR1 = SlowRobot.SlowRobot(robotRad,5)

robots = [r1,slowR1]


# Setup
for robot in robots:
    robot.setPath(np.array([[0.0,0.0],[1.0,1.0],[2.0,2.0]]))
    robot.setPos(np.array((0.0,0.0)))

class Path:
    def __init__(self, points,length):
        self.points = points
        self.length = length

paths = [Path(1,1),Path(1),Path(3),Path(3),Path(9),Path(9)]

sched = PathScheduler(2,paths)

# while True:
    # print(f'Time: {time:.3f}')
    # for index,robot in enumerate(robots):
    #     print(f'Node[{index}] Pos: {robot.pos}')
    #     robot.update(dt)
    # time = time + dt
    # print("---------------------")




    # if(all(robot.pathComplete for robot in robots)):
    #     print("All nodes done")
    #     for index,robot in enumerate(robots):
    #         print(f'Node[{index}] Pos: {robot.pos}')
    #     break


sched.printAssignments()