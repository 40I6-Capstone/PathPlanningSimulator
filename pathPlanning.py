import math;
import numpy as np;
import scipy.interpolate as si
import scipy.spatial as ss



colors = ('b', 'g', 'r', 'c', 'm', 'y', 'k')

class TestShape:
    def __init__(self, centre_x, centre_y, num_sides, radius):
        self.centre = [centre_x, centre_y];
        vertices = [];
        midpoints = [];
        norms = [];

        # calculate the angle of the vector pointing to the centre of the shape from the origin
        base_angle = math.pi + math.atan(centre_y/centre_x);
        # calculate the angle between two vertices 
        d_angle = 2*math.pi/num_sides;

        # calculate the location of the vertices having one point to the origin
        for i in range(num_sides):
            x = centre_x + radius * math.cos((base_angle+i*d_angle)%(2*math.pi));
            y = centre_y + radius * math.sin((base_angle+i*d_angle)%(2*math.pi));
            vertices.append([x,y]);

        #calculate the location of the midpoints and their normal vectors
        for j in range(len(vertices)):
            next = (j+1)%len(vertices);
            x_diff = (vertices[j][0]-vertices[next][0]);
            y_diff = (vertices[j][1]-vertices[next][1]);
            x = (x_diff/2) + vertices[next][0];
            y = (y_diff/2) + vertices[next][1];
            midpoints.append([x,y]);
            
            x_norm = -y_diff/math.sqrt(math.pow(x_diff,2)+math.pow(y_diff,2));
            y_norm = x_diff/math.sqrt(math.pow(x_diff,2)+math.pow(y_diff,2));
            norms.append([x_norm, y_norm]);

        vertices.append(vertices[0])
        self.vertices = np.array(vertices);
        self.midpoints = np.array(midpoints);
        self.norms = np.array(norms);

class Path:
    def __init__(self, points):
        self.points = points;
        self.length = np.sum(np.sqrt(np.diff(points[:,0])**2 + np.diff(points[:,1])**2))       
class PathPlanning:
    def __init__(self, shape:TestShape, crit_rad, ugv_rad, spoke_len):
        self.paths = [];

        # calculate the angle from the origin to the centre of the shape
        base_angle = math.atan(shape.centre[1]/shape.centre[0]);
        # calculate how many paths we need to plan
        num_of_paths = len(shape.midpoints);


        # calculate the angle the ugv's will leave from the origin for its own lane
        d_angle = 2*math.asin((1.05*ugv_rad/2)/crit_rad);

        # calculate the radius of the inscribed circle for the shape
        radius = math.sqrt(math.pow((shape.midpoints[0][0]-shape.midpoints[int(num_of_paths/2)][0]),2)+math.pow((shape.midpoints[0][1]-shape.midpoints[int(num_of_paths/2)][1]),2))/2;
        dist = math.sqrt(math.pow(shape.centre[0],2)+math.pow(shape.centre[1],2));
        
        # number of points for 1st part of the path (in the crit circle)
        nop_1 = 100;
        d_rad = crit_rad/nop_1;

        # number of points for the 3rd part of the path (the spoke length)
        nop_3 = 50;
        d_len_3 = spoke_len/50;


        for i in range(num_of_paths):
            path1 = [[0,0]];
            path2 = [];
            path3 = [];
            cv = [];
            order = 1;

            if(i%2):
                angle_off = i*d_angle;
                shape_i = int(-(i+1)/2);
                index_coef = (i-1)/2 + 0.2;

            else:
                angle_off = -(i + 1)*d_angle;
                shape_i = int(i/2);
                index_coef = i/2 + 0.2;
            
            ctl_ext = self._get_ext_coef(radius, dist, index_coef, order);

            # calculate the points for inside the critical circle
            for j in range(nop_1):
                cr_x = j*d_rad*math.cos((base_angle+angle_off));
                cr_y = j*d_rad*math.sin((base_angle+angle_off));
                path1.append([cr_x,cr_y]);
            # add the end of path 1 to the control points (start anchor)
            cv.append([cr_x,cr_y]);

            # add the extension to the path 1 to the control points (off path)
            cr_x_ext = (crit_rad + ctl_ext[0])*math.cos((base_angle+angle_off));
            cr_y_ext = (crit_rad + ctl_ext[0])*math.sin((base_angle+angle_off));
            cv.append([cr_x_ext,cr_y_ext]);
            
            # add the extension to the spoke path to the control points (off path)
            sp_x_ext = shape.midpoints[shape_i,0] + (spoke_len + ctl_ext[1])*shape.norms[shape_i,0];
            sp_y_ext = shape.midpoints[shape_i,1] + (spoke_len + ctl_ext[1])*shape.norms[shape_i,1];
            cv.append([sp_x_ext,sp_y_ext]);

            #calculate the values for the spoke path
            for k in range(nop_3):
              sp_x = shape.midpoints[shape_i,0] + k*d_len_3*shape.norms[shape_i,0];
              sp_y = shape.midpoints[shape_i,1] + k*d_len_3*shape.norms[shape_i,1];
              path3.append([sp_x,sp_y]);
            # add the end of path 3 to the control points (end anchor)
            cv.append([sp_x,sp_y]);

            # reverse the path so it goes towards the shape
            path3.reverse();

            # calculate the path points for the cubic b spline (path 2)
            cv = np.array(cv);
            path2 = self._b_spline(cv);

            if len(self.paths) >= 2:
                min_path_dist =  ss.distance.cdist(path2, self.paths[-2].points).min();
                while(min_path_dist < 2*ugv_rad):
                    order += 1; 
                    ctl_ext = self._get_ext_coef(radius, dist, index_coef, order);

                    cr_x_ext = (crit_rad + ctl_ext[0])*math.cos((base_angle+angle_off));
                    cr_y_ext = (crit_rad + ctl_ext[0])*math.sin((base_angle+angle_off));
                    cv[1] = np.array([cr_x_ext,cr_y_ext]);

                    sp_x_ext = shape.midpoints[shape_i,0] + (spoke_len + ctl_ext[1])*shape.norms[shape_i,0];
                    sp_y_ext = shape.midpoints[shape_i,1] + (spoke_len + ctl_ext[1])*shape.norms[shape_i,1];
                    cv[2] = np.array([sp_x_ext,sp_y_ext]);

                    path2 = self._b_spline(cv);
                    min_path_dist =  ss.distance.cdist(path2, self.paths[-2].points).min();

            # concatenate the 3 path parts to get the complete path and add it to the path list 
            path = Path(np.array(path1+path2+path3))
            print(path.length)
            self.paths.append(path);
    
    # function to get how much the control points should extend by from the anchor points
    def _get_ext_coef(self, radius, dist, index_coef, order):
            # TODO update how much the control points extends by
            ctl_rad_ext = order*(0.1*radius + 0.05*dist + 0.5*index_coef);
            ctl_spoke_ext = order*(0.2*radius + index_coef);
            return np.array([ctl_rad_ext, ctl_spoke_ext]);

    def _b_spline(self, cv):
        n=1000;
        cv = np.asarray(cv);
        count = cv.shape[0];
        degree = 2;

        # Prevent degree from exceeding count-1, otherwise splev will crash
        degree = np.clip(degree,1,count-1);

        # Calculate knot vector
        kv = np.array([0]*degree + list(range(count-degree+1)) + [count-degree]*degree,dtype='int');

        # Calculate query range
        u = np.linspace(0,(count-degree),n);

        # Calculate result
        return np.array(si.splev(u, (kv,cv.T,degree))).T.tolist();

