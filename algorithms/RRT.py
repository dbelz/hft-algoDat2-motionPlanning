import random
from dijkstra import DijkstraSPF, Graph
import time

class RRT:

    # TODO: Define interface or parent class which defines a basic algorithm
    # TODO: Is it really a good idea to pass the workspace into the algo class?
    def __init__(self, workspace, configspace) -> None:
        
        self.vertex = []
        self.graph = Graph()
        
        self.workspace = workspace
        self.configspace = configspace

