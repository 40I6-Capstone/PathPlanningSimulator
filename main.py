import math;
import numpy as np;
import scipy.interpolate as si

class testShape:
    def __init__(self, centre_x, centre_y, num_sides, radius):
        self.centre = (centre_x, centre_y);
        vertices = [];
        self.midpoints = [];
        self.norms = [];

        base_angle = math.pi + math.atan(centre_x/centre_y);
        d_angle = 2*math.pi/num_sides;

        for i in range(num_sides):
            x = centre_x + radius * math.cos((base_angle+i*d_angle)%(2*math.pi));
            y = centre_y + radius * math.sin((base_angle+i*d_angle)%(2*math.pi));
            vertices.append((x,y));

        for j in range(len(vertices)-1,-1,-1):
            x_diff = (vertices[j-1][0]-vertices[j][0]);
            y_diff = (vertices[j-1][1]-vertices[j][1]);
            x = (x_diff/2) + vertices[j][0];
            y = (y_diff/2) + vertices[j][1];
            self.midpoints.append((x,y));
            
            x_norm = -y_diff/math.sqrt(math.pow(x_diff,2)+math.pow(y_diff,2));
            y_norm = x_diff/math.sqrt(math.pow(x_diff,2)+math.pow(y_diff,2));
            self.norms.append((x_norm, y_norm));
            
class pathPlanning:
    def __init__(self, shape:testShape, crit_rad, ugv_rad, spoke_len):
        base_angle = math.pi + math.atan(shape.centre[0]/shape.centre[1]);
        num_of_paths = len(shape.midpoints);

    def _bspline():
        cv = np.asarray(cv)
        count = cv.shape[0]

        # Prevent degree from exceeding count-1, otherwise splev will crash
        degree = np.clip(degree,1,count-1)

        # Calculate knot vector
        kv = np.array([0]*degree + list(range(count-degree+1)) + [count-degree]*degree,dtype='int')

        # Calculate query range
        u = np.linspace(0,(count-degree),n)

        # Calculate result
        return np.array(si.splev(u, (kv,cv.T,degree))).T    

testShape(30, 35, 4, 20);
