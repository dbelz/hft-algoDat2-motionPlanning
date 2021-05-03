import sys
import numpy as np
import random
from tkinter import messagebox
from dijkstra import DijkstraSPF, Graph

from pprint import pprint as pp
np.set_printoptions(suppress=True,linewidth=np.nan,threshold=sys.maxsize)

class sPRM:

    # TODO: Define interface or parent class which defines a basic algorithm
    # TODO: Is it really a good idea to pass the workspace into the algo class?
    def __init__(self, radius, nr_of_samples, workspace, configspace) -> None:
        
        #edges = {}
        vertex = []
        graph = Graph()
        #self.workspace = workspace
        #self.configspace = configspace
        
        c_init = configspace.initConfig
        c_goal = configspace.goalConfig
        
        # Add start and goal configurations to the vertex data structure
        vertex.append(c_init)
        vertex.append(c_goal)
        
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
                vertex.append((x,y))
            #else:
                # TODO: Do I have to repeat this random sample to have the configured nr_of_samples in the end?
                #print("COLLISION for sample: {} ==> x: {}, y: {}".format(i, x, y))
        
        # Iterate over all valid samples and search for neighbors in a given radius
        print("Searching for possible neighbors of each configuration sample...")
        c = 0
        t = len(vertex)
        for config in vertex:
            c = c + 1
            #print("CHECK for possible neighbor of sample {} of {}".format(c, t))
            for possible_neighbor in vertex:
                dist = self._calculate_distance(config, possible_neighbor)
                if (dist < radius and dist > 0):
                    #print("sample: {}, poss_neighbor: {}, dist: {}".format(config, possible_neighbor, dist))
                    
                    # TODO: Calculate the needed amount of samples according to algo in lecture notes
                    # To keep it simple we use the radius of the robot as a start
                    steps = np.linspace(config, possible_neighbor, round(dist/24), endpoint=False)
                    for step in steps:
                        if (not workspace.isInCollision(int(step[0]), int(step[1]))):
                            #print("POSSIBLE NEIGHBOR FOUND: {}".format(possible_neighbor))
                            graph.add_edge(self._encode_config(config), self._encode_config(possible_neighbor), dist)
                    
        print("NODES:")
        print(str(graph))
        print("--> Number of edges: {}".format(len(graph.get_nodes())))
           
        # Use the Dijkstra algorithm to find the shortest path from c_init to c_goal
        # TODO: What if we have several init and goal configurations?
        dijkstra = DijkstraSPF(graph, self._encode_config(c_init))
        
        print("Computing the shortest path to {} now...".format(self._encode_config(c_goal)))
        try:
            shortest_path = dijkstra.get_path(self._encode_config(c_goal))
        except:
            print("No path can be found from {} to {}, using {} samples and a neighbor search radius of {}".format(c_init, c_goal, nr_of_samples, radius))
            messagebox.showwarning("No path found", "No path can be found from {} to {}, using {} samples and a neighbor search radius of {}".format(c_init, c_goal, nr_of_samples, radius))
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