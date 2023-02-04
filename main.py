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