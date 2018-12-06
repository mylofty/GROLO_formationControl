import os
import numpy as np

deltaT = 0.8  # time increase ratio
time = 0  # time increase

folder = "data"

random_node_filename = "random_nodes.npy"
beacon_node_filename = "beacon_nodes.npy"

dv_distance_result = "dv_distance_result.npy"
TE_parent_filename = "TE_parent.npy"
gradient_descent_result = "gradient_descent_result.npy"
GROLO_result = "GROLO_result.npy"

# formation control
# communication_distance = 6
allow_distance = 4.1
collision_distance = 0.5
deltaV = 0.2
deltaD = np.pi/12




