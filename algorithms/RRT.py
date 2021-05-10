import random
from utils import calculate_distance, decode_config, encode_config, find_nearest_neighbor, find_neighbors, get_cfg_between, is_edge_valid, is_in_collision
from dijkstra import DijkstraSPF, Graph
import time

class RRT:

    # TODO: Define interface or parent class which defines a basic algorithm
    # TODO: Is it really a good idea to pass the workspace into the algo class?
    def __init__(self, workspace, configspace, c_init, c_goal, range, iterations) -> None:
        
        self.vertex = []
        self.graph = Graph()
        
        self.workspace = workspace
        self.configspace = configspace
        
        self.c_init = c_init
        self.c_goal = c_goal
        
        self.vertex.append(c_init)
        # TODO: When also appending g_goal we might run into the prob that
        #       c_goal is found as nearest neighbor. Correct?
        
        self.range = range
        self.iterations = iterations


    # -------------------------------------------------------------------------
    def find_path(self):
        
        start_time = time.perf_counter
        
        env_width = self.workspace.envArray.shape[1] - 1
        env_height = self.workspace.envArray.shape[0] - 1
        
        for _ in range(self.iterations):
            
            r_x = int(random.uniform(0, env_width))
            r_y = int(random.uniform(0, env_height))
        
            #print("random sample: {} ==> x: {}, y: {}".format(i, x, y))
            
            # If the sample is on an obstacle, continue with the next random
            # config sample
            if (is_in_collision(self.workspace, r_x, r_y)):
                continue

            c_rand = (r_x, r_y)
        
            # TODO: What if the "line" between the nearest config and the random config
            #       crosses an obstacle?
            c_near = find_nearest_neighbor(c_rand, self.vertex)
            c_new = get_cfg_between(c_near, c_rand, self.range)
            dist = calculate_distance(c_new, c_near)
            
            if (not is_edge_valid(self.workspace, c_near, c_new, dist)):
                continue
            
            self.vertex.append(c_new)
            self.graph.add_edge(encode_config(c_near), encode_config(c_new), dist)
            
            # Check how close we are to the goal configuration.
            # Assumption: When we are closer to the goal configuration than the
            # given range, we automatically add an edge from the current c_new
            # to c_goal and are finished
            dist = calculate_distance(self.c_goal, c_new)
            if (dist < range):
                self.vertex.append(self.c_goal)
                self.graph.add_edge(encode_config(c_new), encode_config(self.c_goal), dist)
                
                break
        
        dijkstra = DijkstraSPF(self.graph, encode_config(self.c_init))
        path = dijkstra.get_path(encode_config(self.c_goal))
        
        print("Path: ", end="")
        print(" -> ".join(path))
        print("Distance: {}".format(dijkstra.get_distance(encode_config(c_goal))))
        
        solution_path = []
        for cfg in path:
            solution_path.append(decode_config(cfg))     
            
        return solution_path