from RobotTypes.Robot import Robot
import  numpy as np

class IdealRobot(Robot):

    def update(self,dt):
        self.pos = self.path[self.nextPointIndex]
        self.incrementNextPoint()
        np.append(self.posArr, self.pos);
        

        
          