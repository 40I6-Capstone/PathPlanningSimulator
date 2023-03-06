from RobotTypes.Robot import Robot
import  numpy as np


class SlowRobot(Robot):
    def __init__(self, radius,topSpeed):
        super().__init__(radius)
        self.topSpeed = topSpeed

    def computeVelocity(self):
        heading = self.path[self.nextPointIndex] - self.pos
        norm =  np.linalg.norm(heading)
        if(abs(norm) > 0.001):
            velocity = heading / norm * self.topSpeed
            return velocity
        else:
            raise Exception 

    def update(self,dt):
        # Create a vector from current position to next with a magnitude of velocity
        timeLeft = dt;
        if(self.atNextPoint()):
            self.incrementNextPoint()
        while timeLeft > 0 and not(self.pathComplete):
            try:
                velocity = self.computeVelocity()
            except:
                self.incrementNextPoint();
                continue
            timeToNext = self.distToNextPoint()/np.linalg.norm(velocity)
            if timeToNext < timeLeft:
                self.pos = self.pos + velocity * timeToNext
                timeLeft = timeLeft - timeToNext
            else:
                self.pos = self.pos + velocity * timeLeft
                timeLeft = timeLeft -   dt
            if(self.atNextPoint()):
                self.incrementNextPoint()
        self.posArr = np.vstack([self.posArr, self.pos]);
                