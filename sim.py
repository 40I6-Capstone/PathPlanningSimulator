from RobotTypes import *
from time import sleep
import numpy as np
from PathScheduler import PathScheduler
from pathPlanning import (TestShape, PathPlanning)
import ast



class Sim:
    def __init__(self):
        
        np.seterr(all='raise')
        
        #get shape data from file
        # shape_data_file =  open(sys.argv[1], "r");
        # shape_data_file =  open("close_hex.txt", "r");
        shape_data_file =  open("far16.txt", "r");

        shape_data_str = shape_data_file.read();
        shape_data = ast.literal_eval(shape_data_str);

        self.robotRad = 5 # Robot radius in meters
        self.dt = 0.02 # Simulation timestep
        self.simTime = 50 # Total Simulation time
        self.time = np.arange(0, self.simTime, self.dt) # Current Time in simulation
        self.nodeCount = 3
        self.robotInCrit = None;
        self.critQueue = [];
        self.ugvOnSet = [-1] * self.nodeCount;


        # TODO set test shape
        self.shape = TestShape(shape_data["x"], shape_data["y"], shape_data["sides"], shape_data["rad"]);

    def setPath(self, path: PathPlanning):
        self.pathPlan = path;

    def nodeSetup(self):        
        ##############  NODE SETUP  ############## 

        # slowR1 = SlowRobot.SlowRobot(robotRad,10)
        self.robots = []
        for i in range(self.nodeCount):
            self.robots.append(SlowRobot.SlowRobot(self.robotRad,50))

        # Create Scheduler
        self.scheduler = PathScheduler(self.nodeCount,self.pathPlan.paths)
        print(self.scheduler.assignedPathIndexes)

        # Inital path assignments
        for robotIndex, robot in enumerate(self.robots):
            i = 0;
            while(self.pathPlan.paths[self.scheduler.assignedPathIndexes[robotIndex][i]].set in self.ugvOnSet):
                i+=1;
            pathPoints = self.pathPlan.paths[self.scheduler.assignedPathIndexes[robotIndex][i]].points
            self.ugvOnSet[robotIndex] = self.pathPlan.paths[self.scheduler.assignedPathIndexes[robotIndex][i]].set
            self.scheduler.assignedPathIndexes[robotIndex].remove(self.scheduler.assignedPathIndexes[robotIndex][i])
            print(robotIndex, self.ugvOnSet, self.scheduler.assignedPathIndexes[robotIndex][i]);
            robot.setPath(pathPoints)
            # print(pathPoints)
            robot.setPos(pathPoints[0])

    ##############  Simulation  ############## 
    # simEnd = False
    def runSim(self):
        for currTime in self.time:
            # print(f'Time: {self.time:.3f}')
            robot_in_cr = any([robot.checkCriticalRad(self.pathPlan.crit_rad) for robot in self.robots]);
            for index,robot in enumerate(self.robots):
                # Move a robot
                # print(f'Node[{index}] Pos: {robot.pos}')
                # print(f'index {index} is started {robot.started}');
                if(not (robot.started or robot_in_cr)):
                    robot.start();
                    robot_in_cr = True;

                robot.update(self.dt);

                if(robot.stopped and self.robotInCrit == index):
                    if(len(self.critQueue)>0):
                        self.robotInCrit = self.critQueue.pop();
                        self.robots[self.robotInCrit].start();
                    else: 
                        self.robotInCrit = None; 
                
                if(robot.checkCriticalRad(self.pathPlan.crit_rad)):
                    if(self.robotInCrit == None):
                        self.robotInCrit = index;
                    elif(not self.robotInCrit == index):
                        self.critQueue.append(index);
                        robot.stop();
                elif(self.robotInCrit == index):
                    if(len(self.critQueue)>0):
                        self.robotInCrit = self.critQueue.pop();
                        self.robots[self.robotInCrit].start();
                    else: 
                        self.robotInCrit = None;

                
                # Update a nodes path, if the current one is complete
                if(robot.pathComplete and len(self.scheduler.assignedPathIndexes[index]) > 1 ):
                    self.ugvOnSet[index] = -1;
                    i = 0;
                    while(self.pathPlan.paths[self.scheduler.assignedPathIndexes[index][i]].set in self.ugvOnSet):
                        i+=1;
                    pathPoints = self.pathPlan.paths[self.scheduler.assignedPathIndexes[index][i]].points
                    self.ugvOnSet[index] = self.pathPlan.paths[self.scheduler.assignedPathIndexes[index][i]].set
                    self.scheduler.assignedPathIndexes[index].remove(self.scheduler.assignedPathIndexes[index][i])
                    print(index, self.ugvOnSet, self.scheduler.assignedPathIndexes[index][i]);

                    # pathPoints = self.pathPlan.paths[self.scheduler.assignedPathIndexes[index][newPathIndex]].points
                    
                    robot.setPath(pathPoints)
                    robot.setPos(pathPoints[0])
                elif(robot.pathComplete):
                    robot.stop();
            # print("---------------------")


            global simEnd
            # Check if all the robots are done
            if(all(robot.pathComplete for robot in self.robots)):
                # print("All nodes done")
                # for index,robot in enumerate(robots):x
                    # print(f'Node[{index}] Pos: {robot.pos}')
                simEnd = True 
            simEnd = False

    def getPos(self, time):
        result = np.where(abs(self.time - time) < 0.0001);
        if(len(result) == 0): return;
        i = result[0][0];
        for robot in self.robots:
            robot.setPosAtTime(i);




