from RobotTypes import *
from time import sleep
import numpy as np
from PathScheduler import PathScheduler
from pathPlanning import (TestShape, PathPlanning)
import matplotlib.pyplot as plt
import ast
import sys



class Sim:
    def __init__(self):
        
        np.seterr(all='raise')
        
        #get shape data from file
        # shape_data_file =  open(sys.argv[1], "r");
        shape_data_file =  open("close_hex.txt", "r");

        shape_data_str = shape_data_file.read();
        shape_data = ast.literal_eval(shape_data_str);


        # TODO set test shape
        self.shape = TestShape(shape_data["x"], shape_data["y"], shape_data["sides"], shape_data["rad"]);

        # plot shape and midpoints
        # plt.plot(shape.vertices[:,0],shape.vertices[:,1])
        # plt.plot(shape.midpoints[:,0],shape.midpoints[:,1], 'o')

        #get environment data from file
        env_data_file =  open("env_data.txt", "r");
        env_data_str = env_data_file.read();
        env_data = ast.literal_eval(env_data_str);

        # TODO set 
        self.pathPlan = PathPlanning(self.shape, env_data["crit_rad"], env_data["ugv_rad"], env_data["spoke_len"]);
        
        ##############  NODE SETUP  ############## 

        self.robotRad = 0.05 # Robot radius in meters
        self.dt = 0.02 # Simulation timestep
        self.time = 0.0 # Current Time in simulation
        self.simTime = 15 # Total Simulation time
        self.nodeCount = 3

        # slowR1 = SlowRobot.SlowRobot(robotRad,10)
        self.robots = []
        for i in range(self.nodeCount):
            self.robots.append(SlowRobot.SlowRobot(self.robotRad,50))

        # Create Scheduler
        self.scheduler = PathScheduler(self.nodeCount,self.pathPlan.paths)
        print(self.scheduler.assignedPathIndexes)

        # Inital path assignments
        for robotIndex, robot in enumerate(self.robots):
            pathPoints = self.pathPlan.paths[self.scheduler.assignedPathIndexes[robotIndex][0]].points
            robot.setPath(pathPoints,0)
            # print(pathPoints)
            robot.setPos(pathPoints[0])

        # temp
        fig, ax = plt.subplots()

    # def animate(i):
    #     updateRobots()
    #     ax.clear()
    #     ax.plot(shape.vertices[:,0],shape.vertices[:,1])
    #     ax.plot(shape.midpoints[:,0],shape.midpoints[:,1], 'o')
    #     for path in pathPlan.paths:
    #         ax.plot(path.points[:,0],path.points[:,1],'k-');

        
    #     colors = ['r','g','b','c','m','y']
    #     for index, robot in enumerate(robots):
    #         xpoints = []
    #         ypoints = []
    #         xpoints.append(robot.pos[0]) 
    #         ypoints.append(robot.pos[1])
    #         ax.plot(xpoints,ypoints, colors[index%len(colors)]+"x")


    #     #display plot
    #     plt.xlabel('x');
    #     plt.ylabel('y');
    #     fig.gca().set_aspect('equal', adjustable='box');

    ##############  Simulation  ############## 
    # simEnd = False
    def updateRobots(self):
        global time
        # print(f'Time: {time:.3f}')
        for index,robot in enumerate(self.robots):
            # Move a robot
            # print(f'Node[{index}] Pos: {robot.pos}')
            robot.update(self.dt)
            # Update a nodes path, if the current one is complete
            if(robot.pathComplete and robot.pathIndex < len(self.scheduler.assignedPathIndexes[index])-1 ):
                newPathIndex = robot.pathIndex + 1
                pathPoints = self.pathPlan.paths[self.scheduler.assignedPathIndexes[index][newPathIndex]].points
                robot.setPath(pathPoints,newPathIndex)
                robot.setPos(pathPoints[0])

        time = time + self.dt
        # print("---------------------")


        global simEnd
        # Check if all the robots are done
        if(all(robot.pathComplete for robot in self.robots)):
            # print("All nodes done")
            # for index,robot in enumerate(robots):x
                # print(f'Node[{index}] Pos: {robot.pos}')
            simEnd = True 
        simEnd = False



