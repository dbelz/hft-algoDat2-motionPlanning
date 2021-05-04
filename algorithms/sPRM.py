import sys
import numpy as np
import random
from tkinter import messagebox
from dijkstra import DijkstraSPF, Graph
import time
#import multiprocessing as mp
from pprint import pprint as pp

np.set_printoptions(suppress=True,linewidth=np.nan,threshold=sys.maxsize)

class sPRM:

    # TODO: Define interface or parent class which defines a basic algorithm
    # TODO: Is it really a good idea to pass the workspace into the algo class?
    def __init__(self, radius, nr_of_samples, workspace, configspace) -> None:
        
        self.vertex = []
        self.graph = Graph()
        
        self.radius = radius
        self.nr_of_samples = nr_of_samples
        
        self.workspace = workspace
        self.configspace = configspace
        
        c_init = configspace.initConfig
        c_goal = configspace.goalConfig
        
        # Add start and goal configurations to the vertex data structure
        self.vertex.append(c_init)
        self.vertex.append(c_goal)
        
        # Distribute the samples evenly on the room map
        #  - random x and y values
        #  - uniformly distributed
        #    https://en.wikipedia.org/wiki/Continuous_uniform_distribution
        env_width = workspace.envArray.shape[1] - 1
        env_height = workspace.envArray.shape[0] - 1
        #print("env_width: {}, env_height: {}".format(env_width, env_height))
        
        print("Creating random configuration samples...")
        for i in range(1, nr_of_samples):
            x = int(random.uniform(0, env_width))
            y = int(random.uniform(0, env_height))
        
            #print("random sample: {} ==> x: {}, y: {}".format(i, x, y))
            
            # If the sample is not on an obstacle, add it to the vertex data structure
            if (not workspace.isInCollision(x, y)):
                self.vertex.append((x,y))
            #else:
                # TODO: Do I have to repeat this random sample to have the configured nr_of_samples in the end?
                #print("COLLISION for sample: {} ==> x: {}, y: {}".format(i, x, y))
        
        # Iterate over all valid samples and search for neighbors in a given radius
        print("Searching for possible neighbors of each configuration sample...")
        start = time.perf_counter()
        for config in self.vertex:
            self._find_neighbors(config)
        #print("Number of available CPUs: {}".format(mp.cpu_count()))
        #with mp.Pool(processes=4) as pool:
        #    pool.map(find_neighbors, [config for config in self.vertex])
        end = time.perf_counter()
        print("Finding neighbors took {:0.4f} seconds".format(end - start))
                    
        #print("NODES:")
        #print(str(self.graph))
        print("Number of edges in graph: {}".format(len(self.graph.get_nodes())))
           
        # Use the Dijkstra algorithm to find the shortest path from c_init to c_goal
        # TODO: What if we have several init and goal configurations?
        dijkstra = DijkstraSPF(self.graph, self._encode_config(c_init))
        
        print("Computing the shortest path to {} now...".format(self._encode_config(c_goal)))
        try:
            start = time.perf_counter()
            shortest_path = dijkstra.get_path(self._encode_config(c_goal))
            end = time.perf_counter()
            print("Computing the path took {:0.4f} seconds".format(end - start))
        except:
            print("No path can be found from {} to {}, using {} samples and a neighbor search radius of {}".format(c_init, c_goal, nr_of_samples, radius))
            messagebox.showwarning("WARNING: No path found", "No path can be found from {} to {}, using {} samples and a neighbor search radius of {}".format(c_init, c_goal, nr_of_samples, radius))
            return
        
        print("Shortest path:")
        print(" -> ".join(shortest_path))
        
        print("Distance: {}".format(dijkstra.get_distance(self._encode_config(c_goal))))
        
        print("Creating the solution path...")
        # Note: init and goal states are already included
        solution_path = []
        for config in shortest_path:
            solution_path.append(self._decode_config(config))        
        
        print("Displaying solution path...")
        configspace.solutionPath = solution_path
        configspace.drawSpace()
        
        # TODO: Idea - switch to the configuration tab of the GUI automatically
        
        messagebox.showinfo("Solution path computed", "A solution path has been found!")
        
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
    def _find_neighbors(self, config):
        for possible_neighbor in self.vertex:
            dist = self._calculate_distance(config, possible_neighbor)
            if (dist < self.radius and dist > 0):
                #print("sample: {}, poss_neighbor: {}, dist: {}".format(config, possible_neighbor, dist))
                
                # TODO: Calculate the needed amount of samples according to algo in lecture notes
                # To keep it simple we use the radius of the robot as a start
                steps = np.linspace(config, possible_neighbor, round(dist/24), endpoint=False)
                for step in steps:
                    if (not self.workspace.isInCollision(int(step[0]), int(step[1]))):
                        #print("POSSIBLE NEIGHBOR FOUND: {}".format(possible_neighbor))
                        self.graph.add_edge(self._encode_config(config), self._encode_config(possible_neighbor), dist)
        