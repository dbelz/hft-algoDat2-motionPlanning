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
        
        # TODO: Replace by the following code when the UI has been refactored
        #self.c_init = configspace.initConfig
        #self.c_goal = configspace.goalConfig
        
        self.vertex.append(c_init)
        # TODO: When also appending g_goal we might run into the prob that
        #       c_goal is found as nearest neighbor. Correct?
        
        self.range = range
        self.iterations = iterations
        
        self.goal_reached = False


    # -------------------------------------------------------------------------
    def find_path(self):
        
        start_time = time.perf_counter
        
        env_width = self.workspace.envArray.shape[1] - 1
        env_height = self.workspace.envArray.shape[0] - 1
        
        for n_iter in range(self.iterations):
            
            r_x = int(random.uniform(0, env_width))
            r_y = int(random.uniform(0, env_height))
        
            print("random sample: {} ==> x: {}, y: {}".format(n_iter, r_x, r_y))
            
            # If the sample is on an obstacle, continue with the next random
            # config sample
            if (is_in_collision(self.workspace, r_x, r_y)):
                print("  --> skipping, sample has a collision!")
                continue

            c_rand = (r_x, r_y)
        
            # TODO: What if the "line" between the nearest config and the random config
            #       crosses an obstacle?
            c_near = find_nearest_neighbor(c_rand, self.vertex)
            dist = calculate_distance(c_rand, c_near)
            print("  + c_near: {} in distance {}".format(c_near, dist))
            c_new = get_cfg_between(c_near, c_rand, self.range)
            print("  + c_new: ", c_new)
            dist = calculate_distance(c_new, c_near)
            
            # If c_new is on an obstacle, the tree is trapped,
            # continue with the next random config sample
            if (is_in_collision(self.workspace, c_new[0], c_new[1])):
                print("  --> skipping, c_new has a collision!")
                continue
            
            if (not is_edge_valid(self.workspace, c_near, c_new, dist)):
                print("  --> edge between c_new and c_near is not valid!")
                continue
            
            self.vertex.append(c_new)
            self.graph.add_edge(encode_config(c_near), encode_config(c_new), dist)
            print("  --> [{}] edge added from {} to {} with dist {}".format(n_iter, c_near, c_new, dist))
            
            # Check how close we are to the goal configuration.
            # Assumption: When we are closer to the goal configuration than the
            # given range, we automatically add an edge from the current c_new
            # to c_goal and are finished
            dist = calculate_distance(self.c_goal, c_new)
            print("  + distance between c_new and c_goal: ", dist)
            if (dist < self.range):
                print("  --> c_new is close enough to c_goal")
                self.vertex.append(self.c_goal)
                self.graph.add_edge(encode_config(c_new), encode_config(self.c_goal), dist)
                self.goal_reached = True
                break
        
        print("==> numbers of nodes added to graph: ", self.graph.get_number_of_nodes())
        
        solution_path = []
        if self.goal_reached:
            dijkstra = DijkstraSPF(self.graph, encode_config(self.c_init))
            path = dijkstra.get_path(encode_config(self.c_goal))
        
            print("Path: ", end="")
            print(" -> ".join(path))
            print("Distance: {}".format(dijkstra.get_distance(encode_config(self.c_goal))))
        
            for cfg in path:
                solution_path.append(decode_config(cfg))     
        else:
            print("==> NO PATH HAS BEEN FOUND!")
            
            solution_path.append(self.c_init)
            solution_path.append(self.c_goal)
            
        return solution_path