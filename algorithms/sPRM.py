import sys
import numpy as np
import random
from tkinter import messagebox
from dijkstra import DijkstraSPF, Graph
import time
#import multiprocessing as mp
from pprint import pprint as pp

from numpy.lib.financial import _g_div_gp

np.set_printoptions(suppress=True,linewidth=np.nan,threshold=sys.maxsize)

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
        
        # Distribute the samples evenly on the room map
        #  - random x and y values
        #  - uniformly distributed
        #    https://en.wikipedia.org/wiki/Continuous_uniform_distribution
        env_width = self.workspace.envArray.shape[1] - 1
        env_height = self.workspace.envArray.shape[0] - 1
        #print("env_width: {}, env_height: {}".format(env_width, env_height))
        
        print("Creating random configuration samples...")
        for i in range(1, number):
            x = int(random.uniform(0, env_width))
            y = int(random.uniform(0, env_height))
        
            #print("random sample: {} ==> x: {}, y: {}".format(i, x, y))
            
            # If the sample is not on an obstacle, add it to the vertex data structure
            if (not self.workspace.is_in_collision(x, y)):
                self.vertex.append((x,y))


    # -------------------------------------------------------------------------
    def build_neighbor_graph(self, radius):
        
        self.radius = radius
                
        # Iterate over all valid samples and search for neighbors in a given radius
        print("Searching for possible neighbors of each configuration sample...")
        start = time.perf_counter()
        
        count = 0
        total = len(self.vertex)
        for config in self.vertex:
            count += 1
            self._show_progress(int((count/total)*100))
            self._find_neighbors(config)
        print()
        
        # TODO: Parallelize the neighbor search    
        #print("Number of available CPUs: {}".format(mp.cpu_count()))
        #with mp.Pool(processes=4) as pool:
        #    pool.map(find_neighbors, [config for config in self.vertex])
        
        end = time.perf_counter()
        print("Finding neighbors took {:0.4f} seconds".format(end - start))
                    
        #print("NODES:")
        #print(str(self.graph))
        print("Number of nodes in graph: {}".format(len(self.graph.get_nodes())))
        
        
    # -------------------------------------------------------------------------
    def find_path(self, c_init, c_goal):  
       
        print("Initial config: {}".format(c_init))
        print("Goal config: {}".format(c_goal))
        
        # Add start and goal configurations to the vertex data structure
        self.vertex.append(c_init)
        self.vertex.append(c_goal)

        # Search neighbors for the init and the goal configurations and add them
        # to the graph
        self._find_neighbors(c_init)
        self._find_neighbors(c_goal)

        # Use the Dijkstra algorithm to find the shortest path from c_init to c_goal
        # TODO: What if we have several init and goal configurations?
        print("Graph has currently {} nodes".format(len(self.graph.get_nodes())))
        print("Edges from the goal config: ", self.graph.get_adjacent_nodes(self._encode_config(c_goal)))
        dijkstra = DijkstraSPF(self.graph, self._encode_config(c_init))
        
        print("Computing the shortest path from {} to {} now...".format(self._encode_config(c_init), self._encode_config(c_goal)))
        path_found = False
        try:
            shortest_path = dijkstra.get_path(self._encode_config(c_goal))
            messagebox.showinfo("Solution path computed", "A solution path has been found!")
            path_found = True
        except Exception as err:
            print("ERROR: ", err)
            print("No path can be found from {} to {}, using {} samples and a neighbor search radius of {}".format(c_init, c_goal, self.nr_of_samples, self.radius))
            messagebox.showwarning("WARNING: No path found", "No path can be found from {} to {}, using {} samples and a neighbor search radius of {}".format(c_init, c_goal, self.nr_of_samples, self.radius))
        
        solution_path = []
        if (path_found):
        
            print("Shortest path:")
            print(" -> ".join(shortest_path))
            print("Distance: {}".format(dijkstra.get_distance(self._encode_config(c_goal))))
        
            print("Creating the solution path...")
            # Note: init and goal states are already included
            for config in shortest_path:
                solution_path.append(self._decode_config(config))        
        
        else:
            
            # we just add the init and the goal states
            solution_path.append(c_init)
            solution_path.append(c_goal)
        
        # TODO: Return the solution_path here and handle the displaying on the caller side
        #print("Displaying solution path...")
        return solution_path
            
        # TODO: Idea - switch to the configuration tab of the GUI automatically
        # TODO: Idea - draw path directly on the environment picture
        
        
    # -------------------------------------------------------------------------    
    def _calculate_distance(self, a, b):
        return ((b[0] - a[0])**2 + (b[1] - a[1])**2)**(0.5)
    
    # -------------------------------------------------------------------------    
    def _encode_config(self, cfg):
        #print("_encode_config: {} -> {},{}".format(cfg, cfg[0], cfg[1]))
        return "{},{}".format(cfg[0], cfg[1])
    
    # -------------------------------------------------------------------------    
    def _decode_config(self, cfg_string):
        cfg = cfg_string.split(',')
        return (int(cfg[0]), int(cfg[1]))
    
    # -------------------------------------------------------------------------    
    def _find_neighbors(self, config, debug=False):
        for possible_neighbor in self.vertex:
            dist = self._calculate_distance(config, possible_neighbor)
            if (dist < self.radius and dist > 0):
                if debug: print("sample: {}, poss_neighbor: {}, dist: {}".format(config, possible_neighbor, dist))
                
                if (self._is_edge_valid):
                    if debug: print("POSSIBLE NEIGHBOR FOUND: {}".format(possible_neighbor))
                    self.graph.add_edge(self._encode_config(config), self._encode_config(possible_neighbor), dist)
        
    # -------------------------------------------------------------------------
    def _is_edge_valid(self, config, possible_neighbor):
        # TODO: Calculate the needed amount of samples according to algo in lecture notes
        # To keep it simple we use the radius of the robot as a start
        steps = np.linspace(config, possible_neighbor, round(dist/5), endpoint=False)
        for step in steps:
            if (self.workspace.is_in_collision(int(step[0]), int(step[1]))):
                return False
        return True
                
        # -------------------------------------------------------------------------
    def _show_progress(self, progress):
        print("\r[{0:<50}] {1}%".format('#'*int(progress/2), progress), end="\r", flush=True)
        
