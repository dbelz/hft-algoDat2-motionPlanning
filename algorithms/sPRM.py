from collections import deque
import numpy as np


class sPRM:

    def __init__(self, c_init, c_goal, radius, nr_of_samples) -> None:
        
        edges = deque()
        vertex = []
        
        # Add start and goal configurations to the vertex data structure
        vertex.extend(c_init)
        vertex.extend(c_goal)
        
        # Distribute the samples evenly on the room map
        env_width = self.workspace.envArray.shape[0]
        env_height = self.workspace.envArray.shape[1]
        
        dist_step = round(width*height / nr_of_samples)
        
        #for (int i = 0, i < width*height, i = i + nr_of_samples)
        for i in range(0, width*height, dist_step):
            x = i % env_height
            y = i / env_height
            
            # If the sample is not on an obstacle, add it to the vertex data structure
            if (not self.workspace.isInCollision(x, y)):
                vertex.append((x,y))
            
        
        
        