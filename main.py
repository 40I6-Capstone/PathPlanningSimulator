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
from pathPlanning import (TestShape, PathPlanning)

import matplotlib.pyplot as plt
import ast
import sys

#get shape data from file
shape_data_file =  open(sys.argv[1], "r");
shape_data_str = shape_data_file.read();
shape_data = ast.literal_eval(shape_data_str);

# TODO set test shape
shape = TestShape(shape_data["x"], shape_data["y"], shape_data["sides"], shape_data["rad"]);

# plot shape and midpoints
plt.plot(shape.vertices[:,0],shape.vertices[:,1])
plt.plot(shape.midpoints[:,0],shape.midpoints[:,1], 'o')

#get environment data from file
env_data_file =  open("env_data.txt", "r");
env_data_str = env_data_file.read();
env_data = ast.literal_eval(env_data_str);

# TODO set 
pathPlan = PathPlanning(shape, env_data["crit_rad"], env_data["ugv_rad"], env_data["spoke_len"]);

#plot paths
for path in pathPlan.paths:
    plt.plot(path.points[:,0],path.points[:,1],'k-');

#display plot
plt.xlabel('x');
plt.ylabel('y');
plt.gca().set_aspect('equal', adjustable='box');
plt.show();
