from RobotTypes import *
from time import sleep
import numpy as np
from PathScheduler import PathScheduler
from pathPlanning import (TestShape, PathPlanning)
import matplotlib.pyplot as plt
import ast
import sys

#temp
from matplotlib.animation import FuncAnimation

np.seterr(all='raise')

##############  PATH GENERATION  ############## 

#get shape data from file
# shape_data_file =  open(sys.argv[1], "r");
shape_data_file =  open("close_hex.txt", "r");

shape_data_str = shape_data_file.read();
shape_data = ast.literal_eval(shape_data_str);

# TODO set test shape
shape = TestShape(shape_data["x"], shape_data["y"], shape_data["sides"], shape_data["rad"]);

# plot shape and midpoints
# plt.plot(shape.vertices[:,0],shape.vertices[:,1])
# plt.plot(shape.midpoints[:,0],shape.midpoints[:,1], 'o')

#get environment data from file
env_data_file =  open("env_data.txt", "r");
env_data_str = env_data_file.read();
env_data = ast.literal_eval(env_data_str);

# TODO set 
pathPlan = PathPlanning(shape, env_data["crit_rad"], env_data["ugv_rad"], env_data["spoke_len"]);



##############  NODE SETUP  ############## 

robotRad = 0.05 # Robot radius in meters
dt = 0.02 # Simulation timestep
time = 0.0 # Current Time in simulation
simTime = 15 # Total Simulation time
nodeCount = 3

# slowR1 = SlowRobot.SlowRobot(robotRad,10)
robots = []
for i in range(nodeCount):
    robots.append(SlowRobot.SlowRobot(robotRad,50))

# Create Scheduler
scheduler = PathScheduler(nodeCount,pathPlan.paths)
print(scheduler.assignedPathIndexes)

# Inital path assignments
for robotIndex, robot in enumerate(robots):
    pathPoints = pathPlan.paths[scheduler.assignedPathIndexes[robotIndex][0]].points
    robot.setPath(pathPoints,0)
    # print(pathPoints)
    robot.setPos(pathPoints[0])

# temp
fig, ax = plt.subplots()

def animate(i):
    updateRobots()
    ax.clear()
    ax.plot(shape.vertices[:,0],shape.vertices[:,1])
    ax.plot(shape.midpoints[:,0],shape.midpoints[:,1], 'o')
    for path in pathPlan.paths:
        ax.plot(path.points[:,0],path.points[:,1],'k-');

    
    colors = ['r','g','b','c','m','y']
    for index, robot in enumerate(robots):
        xpoints = []
        ypoints = []
        xpoints.append(robot.pos[0]) 
        ypoints.append(robot.pos[1])
        ax.plot(xpoints,ypoints, colors[index%len(colors)]+"x")


    #display plot
    plt.xlabel('x');
    plt.ylabel('y');
    fig.gca().set_aspect('equal', adjustable='box');

##############  Simulation  ############## 
simEnd = False
def updateRobots():
    global time
    # print(f'Time: {time:.3f}')
    for index,robot in enumerate(robots):
        # Move a robot
        # print(f'Node[{index}] Pos: {robot.pos}')
        robot.update(dt)
        # Update a nodes path, if the current one is complete
        if(robot.pathComplete and robot.pathIndex < len(scheduler.assignedPathIndexes[index])-1 ):
            newPathIndex = robot.pathIndex + 1
            pathPoints = pathPlan.paths[scheduler.assignedPathIndexes[index][newPathIndex]].points
            robot.setPath(pathPoints,newPathIndex)
            robot.setPos(pathPoints[0])

    time = time + dt
    # print("---------------------")


    global simEnd
    # Check if all the robots are done
    if(all(robot.pathComplete for robot in robots)):
        # print("All nodes done")
        # for index,robot in enumerate(robots):x
            # print(f'Node[{index}] Pos: {robot.pos}')
        simEnd = True 
    simEnd = False

# while time < simTime:
ani = FuncAnimation(fig,animate, frames=int(simTime/dt), interval=20, repeat=False)

plt.show()


