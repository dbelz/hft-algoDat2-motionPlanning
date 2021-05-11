import random
from dijkstra import DijkstraSPF, Graph
import time
from pprint import pprint as pp
from utils import encode_config, decode_config, find_neighbors, is_in_collision, show_info, show_progress, show_warning

class sPRM:

    # TODO: Define interface or parent class which defines a basic algorithm
    # TODO: Is it really a good idea to pass the workspace into the algo class?
    def __init__(self, workspace, configspace) -> None:
        
        self.vertex = []
        self.graph = Graph()
        
        self.workspace = workspace
        self.configspace = configspace
        
    # -------------------------------------------------------------------------
    def distribute_configuration_samples(self, number):
        
        
        self.nr_of_samples = number
        
        env_width = self.workspace.envArray.shape[1] - 1
        env_height = self.workspace.envArray.shape[0] - 1
        #print("env_width: {}, env_height: {}".format(env_width, env_height))
        
        print("[INF] Creating random configuration samples...")
        start_time = time.perf_counter()
        
        for _ in range(number):
            x = int(random.uniform(0, env_width))
            y = int(random.uniform(0, env_height))
        
            #print("random sample: {} ==> x: {}, y: {}".format(i, x, y))
            
            # If the sample is not on an obstacle, add it to the vertex data structure
            if (not is_in_collision(self.workspace, x, y)):
                self.vertex.append((x,y))
                
        end_time = time.perf_counter()
        print("[PERF] Duration of config sample distribution: {:0.4f} seconds".format(end_time - start_time))


    # -------------------------------------------------------------------------
    def build_neighbor_graph(self, radius):
        
        self.radius = radius
                
        # Iterate over all valid samples and search for neighbors in a given radius
        print("[INF] Searching for possible neighbors of each configuration sample...")
        
        count = 0
        total = len(self.vertex)
        
        start = time.perf_counter()
        
        for config in self.vertex:
            count += 1
            show_progress(int((count/total)*100))
            find_neighbors(self.workspace, config, self.vertex, self.graph, radius)
        print()
        
        end = time.perf_counter()
        print("[PERF] Duration of finding neighbors: {:0.4f} seconds".format(end - start))
        #print("Number of nodes in graph: {}".format(len(self.graph.get_nodes())))
        
        
    # -------------------------------------------------------------------------
    def find_path(self, c_init, c_goal):  
       
        print("[POS] sPRM path - c_init: {}, c_goal: {}".format(c_init, c_goal))
        
        start_time = time.perf_counter()
        
        # Add start and goal configurations to the vertex data structure
        self.vertex.append(c_init)
        self.vertex.append(c_goal)

        # Search neighbors for the init and the goal configurations and add them
        # to the graph
        find_neighbors(self.workspace, c_init, self.vertex, self.graph, self.radius)
        find_neighbors(self.workspace, c_goal, self.vertex, self.graph, self.radius, reverse=True)

        # Use the Dijkstra algorithm to find the shortest path from c_init to c_goal
        dijkstra = DijkstraSPF(self.graph, encode_config(c_init))
        
        print("[INF] Computing the shortest path from {} to {} now...".format(
            encode_config(c_init), encode_config(c_goal)))
        path_found = False
        try:
            shortest_path = dijkstra.get_path(encode_config(c_goal))
            end_time = time.perf_counter()
            print("[PERF] Duration of finding the shortest path: {:0.4f} seconds".format(end_time - start_time))
        
            show_info("Solution path has been found")
            path_found = True
        except:
            show_warning("No path can be found from {} to {}!".format(c_init, c_goal))
        
        solution_path = []
        if (path_found):
        
            print("[POS] Shortest path: ", end="")
            print(" -> ".join(shortest_path))
            print("[POS] Distance: {}".format(dijkstra.get_distance(encode_config(c_goal))))
        
            # Note: init and goal states are already included
            for config in shortest_path:
                solution_path.append(decode_config(config))        
        
        else:
            
            # we just add the init and the goal states
            solution_path.append(c_init)
            solution_path.append(c_goal)
        
        return solution_path
            
        
        
