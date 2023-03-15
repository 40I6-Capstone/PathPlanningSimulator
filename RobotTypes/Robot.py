from abc import ABC, abstractmethod
from math import *
import numpy as np

class Robot(ABC):

    def __init__(self, radius):
        self.pos = np.array([0,0])
        self.heading = 0
        self.path = np.array([[0,0]])
        self.pathIndex = 0
        self.radius = radius
        self.nextPointIndex = 1
        self.isReturning = False
        self.pathComplete = False
        self.posArr = np.array([0,0]);
        self.started = False;
        self.stopped = True;



    # Update the robots position, dt seconds into the future
    @abstractmethod
    def update(self,dt):
        pass

    def reset(self):
        self.pathComplete = False
        self.isReturning = False
        self.started = False
        self.nextPointIndex = 1
        self.heading = 0
        

    # Set current position, passed as in a numpy array
    def setPos(self,point):
        self.pos = point
        self.reset()
    # Set the path being tracked, passed as a 2d numpy array
    def setPath(self,path,pathIndex):
        self.path = path
        self.pathIndex = pathIndex
        self.reset()
        
    def setPosAtTime(self, i):
        self.pos = self.posArr[i];
    

    # Compute the distance between two points as a scalar distance
    def calcDist(self,p1,p2):
        # return sqrt((p2[0]-p1[0])**2 + (p2[1] - p1[1])**2)
        return np.linalg.norm(p2-p1)

    # Distance to next point
    def distToNextPoint(self):
        return self.calcDist(self.path[self.nextPointIndex],self.pos)

    # Increment the next tracked point
    def incrementNextPoint(self):
        if not(self.isReturning):
            self.nextPointIndex = self.nextPointIndex + 1
            if (self.nextPointIndex >= len(self.path)):
                self.isReturning = True
                self.nextPointIndex = len(self.path)-2
        else:
            self.nextPointIndex = self.nextPointIndex - 1
            if (self.nextPointIndex < 0):
                self.nextPointIndex = 0
                self.pathComplete = True
    # Return if robot is at the tracked point
    def atNextPoint(self):
        return True if self.distToNextPoint() < 0.001 else False

    # Get the point along the path that the robot is closest to
    def getClosestPoint(self):
        closestDist = self.calcDist(self.path[0],self.pos)
        closestPoint = self.path[0]
        for point in self.path:
            dist = self.calcDist(point, self.pos);
            if dist < closestDist:
                closestPoint = point

        return closestPoint;

    def getDistanceTo(self,point):
        return self.calcDist(point,self.pos)

    def checkCriticalRad(self, criticalRad):
        return True if self.getDistanceTo(np.array([0,0])) < criticalRad and self.started else False

    def checkCollision(self, robots):
        for robot in robots:
            dist = self.getDistanceTo(robot.pos);
            if dist < self.radius*2 and dist != 0.0:
                return True;
        return False;