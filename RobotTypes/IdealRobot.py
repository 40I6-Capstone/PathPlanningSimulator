from RobotTypes.Robot import Robot

class IdealRobot(Robot):

    def update(self,dt):
        self.pos = self.path[self.nextPointIndex]
        self.incrementNextPoint()
        

        
          